#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node
{
public:
  PreApproach()
  : Node("pre_approach"),
    obstacle_(0.4),
    degrees_(-90.0),
    forward_speed_(0.2),
    angular_speed_(0.5),
    rotate_time_(0.0),
    invalid_scan_count_(0),
    state_(State::WAITING_FOR_SCAN)
  {
    declare_parameter<double>("obstacle", obstacle_);
    declare_parameter<double>("degrees", degrees_);

    obstacle_ = get_parameter("obstacle").as_double();
    degrees_ = get_parameter("degrees").as_double();

    const double target_angle_rad = degrees_ * kPi / 180.0;
    rotate_time_ = std::abs(target_angle_rad) / std::abs(angular_speed_);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(
      100ms,
      std::bind(&PreApproach::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "pre_approach started: obstacle=%.2f m, degrees=%.2f",
      obstacle_,
      degrees_);
  }

private:
  enum class State {
    WAITING_FOR_SCAN,
    MOVING_FORWARD,
    STOP_BEFORE_ROTATE,
    ROTATING,
    DONE,
    SAFE_STOP
  };

  static constexpr double kPi = 3.14159265358979323846;

  bool get_front_distance(
    const sensor_msgs::msg::LaserScan & scan,
    double window_degrees,
    double & front_distance)
  {
    
    std::vector<double> valid_ranges;

    // Convert half window to radians.
    double half_window = window_degrees / 2 * kPi / 180;
    
    // Loop angle from -half_window to +half_window.
    for (double i = -half_window; i < half_window; i += scan.angle_increment)
    {
      // Convert angle to index using angle_min and angle_increment.
      int index = static_cast<int>(std::round((i - scan.angle_min)/scan.angle_increment));
      
      // Skip out-of-range indices.
      if (index < 0 || index >= static_cast<int>(scan.ranges.size())) {
        continue;
      }
      
      double distance = scan.ranges[index];
      
      // Skip non-finite or out-of-range distances.
      if (!std::isfinite(distance) || distance < scan.range_min || distance > scan.range_max) {
        continue;
      }

      valid_ranges.push_back(distance);
      
    }

    // Use the minimum valid distance as front_distance
    if (!valid_ranges.empty()) {
      front_distance = *std::min_element(valid_ranges.begin(), valid_ranges.end());
      return true;
    }
    
    return false;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double distance = 0.0;

    if (get_front_distance(*msg, 10.0, distance)) {
      front_distance_ = distance;
      last_valid_scan_time_ = now();
      invalid_scan_count_ = 0;
    } else {
      ++invalid_scan_count_;
    }
  }

  void timer_callback()
  {
    // implement the state machine:
    switch(state_) {
      // WAITING_FOR_SCAN -> MOVING_FORWARD or STOP_BEFORE_ROTATE
      case State::WAITING_FOR_SCAN: {  
        publish_stop();

        if (!front_distance_.has_value()) {
          return;
        }

        if (front_distance_.value() > obstacle_) {
          state_ = State::MOVING_FORWARD;
          return;
        }

        stop_start_time_ = this->now();
        state_ = State::STOP_BEFORE_ROTATE;
        return;
      }

      // MOVING_FORWARD -> STOP_BEFORE_ROTATE when front_distance <= obstacle
      case State::MOVING_FORWARD: {
        if (!check_runtime_safety()) {
          return;
        }

        publish_forward();
         
        if (front_distance_.value() <= obstacle_) {
          publish_stop();
          stop_start_time_ = this->now();
          state_ = State::STOP_BEFORE_ROTATE;
        }
          
        return;
      }
        
      // STOP_BEFORE_ROTATE -> ROTATING or DONE
      case State::STOP_BEFORE_ROTATE: {
        if (!check_runtime_safety()) {
          return;
        }

        publish_stop();
        double elapsed_stop = (this->now() - stop_start_time_).seconds();
        
        if (elapsed_stop < 0.2) {
          return;
        }

        if (std::abs(degrees_) < 1e-6) {
          state_ = State::DONE;
          return;
        }

        rotation_start_time_ = this->now();
        state_ =  State::ROTATING;
        return;
      }

      // ROTATING -> DONE after rotate_time_
      case State::ROTATING: {
        if (!check_runtime_safety()) {
          return;
        }

        double elapsed = (this->now() -rotation_start_time_).seconds();
        if (elapsed < rotate_time_) {
          publish_rotate();
        }
        else {
          publish_stop();
          state_ = State::DONE;
        }
        
        return;
      }

      // SAFE_STOP and DONE should publish_stop().
      case State::SAFE_STOP: {
        publish_stop();
        return;
      }

      case State::DONE: {
        publish_stop();
        return;
      }
    }
      
  }

  bool check_runtime_safety()
  {
    if (!front_distance_.has_value()) {
      enter_safe_stop("front distance is unavailable");
      return false;
    }

    if ((now() - last_valid_scan_time_).seconds() > 1.0) {
      enter_safe_stop("latest valid scan is older than 1.0 seconds");
      return false;
    }

    if (invalid_scan_count_ >= 10) {
      enter_safe_stop("too many consecutive invalid scan windows");
      return false;
    }

    return true;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
  }

  void publish_forward()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = forward_speed_;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
  }

  void publish_rotate()
  {
    geometry_msgs::msg::Twist cmd;
    const double direction = degrees_ >= 0.0 ? 1.0 : -1.0;
    cmd.linear.x = 0.0;
    cmd.angular.z = direction * std::abs(angular_speed_);
    cmd_vel_pub_->publish(cmd);
  }

  void enter_safe_stop(const std::string & reason)
  {
    safety_stop_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "SAFE_STOP: %s", reason.c_str());
    state_ = State::SAFE_STOP;
    publish_stop();
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double obstacle_;
  double degrees_;
  double forward_speed_;
  double angular_speed_;
  double rotate_time_;

  std::optional<double> front_distance_;
  rclcpp::Time last_valid_scan_time_;
  int invalid_scan_count_;

  State state_;
  rclcpp::Time rotation_start_time_;
  rclcpp::Time stop_start_time_;
  std::string safety_stop_reason_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}
