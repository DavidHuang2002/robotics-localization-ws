# Requirements:
#   A realsense D400 series
#   Install realsense2 ros2 package (refactor branch)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py align_depth:=true
#   $ ros2 launch rtabmap_ros realsense_d400.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameters_map = [
        {
            "frame_id": "base_link",
            "subscribe_depth": True,
            "approx_sync": True,
            # Set publish tf to true for individual RTABMAP testing
            "publish_tf": True,
            "wait_for_transform": 1.0,
            "odom_frame_id": "odom",  
            "cloud_noise_filtering_min_neighbors": "2",
            "cloud_noise_filtering_radius": "0.05",
            "Rtabmap/StartNewMapOnLoopClosure": False,
            "optimize_from_graph_end": False,
            "queue_size": 5,
        }
    ]

    parameters_odom = [
        {
            "frame_id": "base_link",
            "subscribe_depth": True,
            "approx_sync": True,
            # Set publish tf to true for individual RTABMAP testing
            "publish_tf": True,
            "wait_for_transform": 1.0,
            "Odom/Strategy": "1",
            "Vis/CorType": "1",
            "odom_frame_id": "odom",
            "Odom/ResetCountdown": "10",
            "queue_size": 5,
            "GFTT/MinDistance": "10",
            "Odom/GuessMotion": "1",
            "OdomF2M/MaxSize": "1000",
            "Vis/MaxFeatures": "600",
        }
    ]

    remappings = [
        ("rgb/image", "/camera2/color/image_raw"),
        ("rgb/camera_info", "/camera2/color/camera_info"),
        ("depth/image", "/camera2/depth/image_rect_raw"), #Change to 2 maybe 
    ]

    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"),
            # Nodes to launch
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="screen",
                parameters=parameters_odom,
                remappings=remappings,
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=parameters_map,
                remappings=remappings,
                arguments=["-d"],
                # -d will delete the database before starting, otherwise the previous mapping session is loaded
            ),
            
    
        ]
    )
