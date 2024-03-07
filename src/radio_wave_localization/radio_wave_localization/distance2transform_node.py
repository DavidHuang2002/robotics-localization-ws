import rclpy
from rclpy.node import Node
import std_msgs
import geometry_msgs
from localization_algorithm.radio_wave_to_transform import radio_wave_find_transform


berm_length = 2.0
berm_width = 2.0

class Distance2TransformNode(Node):
    def __init__(self):
        super().__init__("distance2transform")
        self.get_logger().info("distance2transform node started")

        radio_distances_topic = "/radio_distances"
        self.radio_distances_subscription = self.create_subscription(
            std_msgs.msg.Float64MultiArray,
            radio_distances_topic,
            # RadioDistances,
            self.radio_dsstances_callback,
            10
        )

    
    def distance_callback(self, msg):
        self.get_logger().info("Received radio distances: %s" % msg)
        d1, d2, d3 = msg.data
        

        radio_wave_find_transform(
            (d1, d2, d3),
            (d1, d2, d3),
            berm_length,
            berm_width,
            0.1,
            0.1
        )

def main(args=None):
    rclpy.init(args=args)
    node = Distance2TransformNode()
    rclpy.spin(node)
    rclpy.shutdown()