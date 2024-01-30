# Requirements:
#   A realsense D400 series
#   Install realsense2 ros2 package (refactor branch)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py align_depth:=true
#   $ ros2 launch rtabmap_ros realsense_d400.launch.py

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    parameters_map = [
        {
            "frame_id": "base_link",
            "subscribe_depth": True,
            "approx_sync": True,
            # Set publish tf to true for individual RTABMAP testing
            "publish_tf": False,
            "wait_for_transform": 1.0,
            "odom_frame_id": "odom",
        }
    ]

    parameters_odom = [
        {
            "frame_id": "base_link",
            "subscribe_depth": True,
            "approx_sync": True,
            # Set publish tf to true for individual RTABMAP testing
            "publish_tf": False,
            "wait_for_transform": 1.0,
            "odom_frame_id": "odom",
        }
    ]

    remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_rect_raw"),
    ]

    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"),
            # Nodes to launch
            Node(
                package="rtabmap_slam",
                node_executable="rtabmap",
                output="screen",
                parameters=parameters_map,
                remappings=remappings,
                arguments=["-d"],
                # -d will delete the database before starting, otherwise the previous mapping session is loaded
            ),
        ]
    )
