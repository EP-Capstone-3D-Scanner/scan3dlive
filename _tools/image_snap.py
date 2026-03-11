import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageGrabber(Node):
    def __init__(self):
        super().__init__('image_grabber')
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_cropped',  # Change this to your topic name
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Save as PNG
        file_name = "snapshot.png"
        cv2.imwrite(file_name, cv_image)
        
        self.get_logger().info(f'Saved image to {file_name}')
        
        # Shutdown after receiving one image
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ImageGrabber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()