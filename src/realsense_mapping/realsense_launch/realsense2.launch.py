import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("realsense_mapping"),
        "config",
        "realsense_params.yaml",
    )

    log_level = "info"

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="realsense2_camera",
                namespace="camera2",
                name="realsense2_camera",
                executable="realsense2_camera_node",
                parameters=[config],
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                emulate_tty=True,
            ),
            
        ]
    )
