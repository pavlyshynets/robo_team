import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rosbridge_launch_file = os.path.join(
        FindPackageShare("rosbridge_server").find("rosbridge_server"),
        "launch",
        "rosbridge_websocket_launch.xml"
    )

    return LaunchDescription([
        Node(
            package="serial_communication",
            executable="serial_node",
            name="serial_node",
            parameters=["src/serial_communication/config/params.yaml"]
        ),
        Node(
            package="teleop_to_serial",
            executable="teleop_to_serial_node",
            name="teleop_to_serial"
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch_file)
        )
    ])

