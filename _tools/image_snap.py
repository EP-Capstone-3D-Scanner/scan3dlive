#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime

class ImageGrabber(Node):
    def __init__(self):
        super().__init__('image_grabber')
        # Define your topic here
        self.topic_name = '/camera1/image_cropped'
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info(f'Waiting for an image on {self.topic_name}...')

    def listener_callback(self, msg):
        self.get_logger().info('Image received! Processing...')
        
        try:
            # 1. Try standard 8-bit conversion, requesting rgb8 to fix the color swap
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'bgr8 conversion failed: {e}. Trying passthrough...')
            # 2. Fallback for non-standard, depth, or monochrome images
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Handle 16-bit depth images
            if cv_image.dtype == 'uint16':
                self.get_logger().info('Scaling 16-bit image to 8-bit...')
                cv_image = (cv_image / 256).astype('uint8')
            # Handle 32-bit float images
            elif cv_image.dtype == 'float32':
                self.get_logger().info('Normalizing float32 image...')
                cv2.normalize(cv_image, cv_image, 0, 255, cv2.NORM_MINMAX)
                cv_image = cv_image.astype('uint8')

        # 3. Create a unique filename with a timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_name = f"snapshot_{timestamp}.png"
        
        # 4. Save the image to the current directory
        success = cv2.imwrite(file_name, cv_image)
        
        if success:
            self.get_logger().info(f'Success! Saved image to {file_name}')
        else:
            self.get_logger().error('Failed to save image. Check folder permissions or image channels.')

        # 5. Shut down the node immediately after saving
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageGrabber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # Allow clean exit via Ctrl+C
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()