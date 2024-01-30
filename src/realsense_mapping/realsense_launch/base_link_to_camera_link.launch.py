import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                node_name="base_camera_tf_publisher",
                remappings=[
                    ("depth", "/camera/depth/image_rect_raw"),
                    ("depth_camera_info", "/camera/depth/camera_info"),
                ],
                arguments=[
                    "0.24", 
                    "-0.2",
                    "0.52", #0.68
                    "0.0",
                    "0.52", #.33
                    "1.57",
                    "base_link",
                    "camera_link",
                    
                    # "0.35", ##original .50
                    # "-0.15", # -.05
                    # "0.0", # changed to .55 from .45
                    # "0",
                    # "0", #original 0
                    # "0.0",#1.57
                    # "base_link",
                    # "camera_link",
                ],
            ),
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     node_name="base_camera_tf_publisher_MO",
            #     remappings=[
            #         ("depth", "/camera/depth/image_rect_raw"),
            #         ("depth_camera_info", "/camera/depth/camera_info"),
            #     ],
            #     arguments=[
            #         "0.0", 
            #         "0.0",
            #         "0.0", #0.68
            #         "0.0",
            #         "0.0", #.33
            #         "0.0",
            #         "map",
            #         "odom",
                    
            #         # "0.35", ##original .50
            #         # "-0.15", # -.05
            #         # "0.0", # changed to .55 from .45
            #         # "0",
            #         # "0", #original 0
            #         # "0.0",#1.57
            #         # "base_link",
            #         # "camera_link",
            #     ],
            # ),
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     node_name="base_camera_tf_publisherOB",
            #     remappings=[
            #         ("depth", "/camera/depth/image_rect_raw"),
            #         ("depth_camera_info", "/camera/depth/camera_info"),
            #     ],
            #     arguments=[
            #         "0.0", 
            #         "0.0",
            #         "0.0", #0.68
            #         "0.0",
            #         "0.0", #.33
            #         "0.0",
            #         "odom",
            #         "base_link",
                    
            #         # "0.35", ##original .50
            #         # "-0.15", # -.05
            #         # "0.0", # changed to .55 from .45
            #         # "0",
            #         # "0", #original 0
            #         # "0.0",#1.57
            #         # "base_link",
            #         # "camera_link",
            #     ],
            # ),
        ]
    )
