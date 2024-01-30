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
                node_name="base_camera_tf_publisher2",
                remappings=[
                    ("depth", "/camera2/depth/image_rect_raw"),
                    ("depth_camera_info", "/camera2/depth/camera_info"),
                ],
                arguments=[
                    "0.0", # -.24
                    "-.12", # -0.089
                    "0.0", #0.7 0.5?
                    "3.14", #yaw 3.14
                    "0.0", #pitch .3 this shit buggy af no cap look up what this means
                    "0.0", #roll 1.57 CHANGE THIS BAcK FOR ROBOT
                    "base_link",
                    "camera2_link",
                    
                    # "0.35", ##original .50
                    # "-0.15", # -.05
                    # "0.0", # changed to .55 from .45
                    # "0",
                    # "0", #original 0
                    # "0.0",#1.57
                    # "base_link",
                    # "camera_link",
                ],
            )
        ]
    )
