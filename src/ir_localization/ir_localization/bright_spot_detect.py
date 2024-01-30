import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2


class BrightSpotDetectNode(Node):
    def __init__(self):
        super().__init__("bright_spot_detect")


        # self.bright_spot_loc = self.create_publisher(Point, )
        self.ir2_subscription= self.create_subscription(
            Image, 
            "/camera/infra2/image_rect_raw", 
            self.ir2_image_callback, 
            10
        )

        
        self.get_logger().info("bright_spot_detect node started")

        self.bridge = CvBridge()
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        self.get_logger().info("params are %s" % params)
        # Set up your specific parameters here
        # Accessing parameter values directly from the params object
        print("Min Threshold: ", params.minThreshold)
        print("Max Threshold: ", params.maxThreshold)
        
        self.detector = cv2.SimpleBlobDetector_create(self.adjustBrightSpotParam(params))


    def ir2_image_callback(self, msg: Image):
        # image is 640 by 480 (w by h)
               
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Apply binary thresholding for bright spot detection
        _, binary_image = cv2.threshold(current_frame, 200, 255, cv2.THRESH_BINARY)

        # Detect blobs
        keypoints = self.detector.detect(binary_image)
        self.process_key_points(keypoints, current_frame)
            

    def process_key_points(self, keypoints, current_frame):
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(current_frame, keypoints, None, (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # Log or process the keypoints
        # self.get_logger().info('Detected {} blobs'.format(len(keypoints)))
        # For each detected blob, log its coordinate (x,y)
        for i, keyPoint in enumerate(keypoints):
            x = keyPoint.pt[0]  # x coordinate of blob
            y = keyPoint.pt[1]  # y coordinate of blob
            self.get_logger().info(f'Blob {i}: Position ({x},{y})')

        # Display the output image with detected blobs (optional)
        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(1)

    # @staticmethod
    # def save_ir_snap_shot(data: ):
    #     pass
    @staticmethod
    def adjustBrightSpotParam(params):
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        params.minThreshold = 240
        params.maxThreshold = 256

        params.filterByArea = True
        params.minArea = 5  # Depending on the size of the LED in pixels
        # params.maxArea = 500  # Adjust as necessary

        # params.filterByCircularity = True
        # params.minCircularity = 0.8

        # params.filterByConvexity = True
        # params.minConvexity = 0.9

        # params.filterByInertia = True
        # params.minInertiaRatio = 0.5

        params.blobColor = 255

        return params


def main(args=None):
    rclpy.init(args=args)
    node = BrightSpotDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()