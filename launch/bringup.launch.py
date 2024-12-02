from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    bcap_service = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("bcap_service"), "launch/bcap_service.launch.py"]
                )
            ]
        ),
        launch_arguments={"model": "cobotta", "ip_address": "192.168.0.102"}.items(),
    )
    ld.add_action(bcap_service)

    cobotta_pro_control = Node(
        package="cobotta_pro_control",
        executable="cobotta_pro_control",
        name="cobotta_pro_control",
        output="log",
    )
    ld.add_action(cobotta_pro_control)
    return ld
