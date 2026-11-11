from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    dashboard = Node(package="rqt_gui", executable="rqt_gui", output="screen",
                     condition=IfCondition(LaunchConfiguration("gui")),
                     arguments=["--standalone", "sensor_gui/SensorsMotors", "--args",
                                "--topic-prefix", LaunchConfiguration("topic_prefix")])
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("simulate", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("topic_prefix", default_value="/lab"),
        Node(package="sensor_gui", executable="serial_bridge", output="screen", parameters=[{
            "port": LaunchConfiguration("port"),
            "simulate": ParameterValue(LaunchConfiguration("simulate"), value_type=bool),
            "topic_prefix": LaunchConfiguration("topic_prefix"),
        }]),
        RegisterEventHandler(OnProcessExit(target_action=dashboard, on_exit=[
            EmitEvent(event=Shutdown(reason="Dashboard closed"))])),
        dashboard,
    ])
