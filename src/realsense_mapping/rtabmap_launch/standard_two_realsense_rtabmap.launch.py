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
            "publish_tf": False,
            "wait_for_transform": 5.0,
            "odom_frame_id": "odom",  
            # "approx_sync_max_interval": 0.02,
            # "tf_delay": 0.1,
            # "tf_tolerance": 0.01,
            "cloud_noise_filtering_min_neighbors": "2",
            "cloud_noise_filtering_radius": "0.05",
            "Rtabmap/StartNewMapOnLoopClosure": "false",
            "optimize_from_graph_end": False,
            "queue_size": 20,
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
            # "approx_sync_max_interval": 0.02,
            "Odom/Strategy": "1",
            "Odom/ResetCountdown": "1",
            "Vis/CorType": "1",
            "odom_frame_id": "odom",
            "OdomF2M/MaxSize": "1000",
            "Vis/MaxFeatures": "1000",
            "OdomF2M/MaxNewFeatures": "1000",
            "Vis/MinInliers": "0",
        }
    ]

    # remappings = [
    #     ("rgb/image", "/camera/color/image_raw"), #cam/color/image
    #     ("rgb/camera_info", "/camera/color/camera_info"), #/cam/info
    #     ("depth/image", "/camera/depth/image_rect_raw"), #/cam/depth/image
    # ]

    remappings = [
        ("rgb/image", "/cam/color/image"),
        ("rgb/camera_info", "/cam/info"),
        ("depth/image", "/cam/depth/image"),
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
