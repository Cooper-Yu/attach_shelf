from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    obstacle = LaunchConfiguration("obstacle")
    degrees = LaunchConfiguration("degrees")
    forward_speed = LaunchConfiguration("forward_speed")
    angular_speed = LaunchConfiguration("angular_speed")
    rotation_scale = LaunchConfiguration("rotation_scale")
    use_rviz = LaunchConfiguration("use_rviz")

    rviz_config = PathJoinSubstitution([
        FindPackageShare("attach_shelf"),
        "rviz",
        "pre_approach.rviz",
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    pre_approach_node = Node(
        package="attach_shelf",
        executable="pre_approach",
        name="pre_approach",
        output="screen",
        parameters=[{
            "obstacle": ParameterValue(obstacle, value_type=float),
            "degrees": ParameterValue(degrees, value_type=float),
            "forward_speed": ParameterValue(forward_speed, value_type=float),
            "angular_speed": ParameterValue(angular_speed, value_type=float),
            "rotation_scale": ParameterValue(rotation_scale, value_type=float),
        }],
    )

    # When the control node finishes, shut down the whole launch system so RViz
    # does not keep the program alive.
    shutdown_when_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=pre_approach_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason="pre_approach completed")),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("obstacle", default_value="0.4"),
        DeclareLaunchArgument("degrees", default_value="-90.0"),
        DeclareLaunchArgument("forward_speed", default_value="0.4"),
        DeclareLaunchArgument("angular_speed", default_value="0.5"),
        DeclareLaunchArgument("rotation_scale", default_value="0.5"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        rviz_node,
        pre_approach_node,
        shutdown_when_complete,
    ])
