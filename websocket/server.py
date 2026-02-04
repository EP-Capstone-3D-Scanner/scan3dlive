from std_msgs.msg import String
from geometry_msgs.msg import PointXYZ
import rclpy
import rclpy.executors
import rclpy.serialization
import asyncio
import queue
import concurrent.futures

ws_msg_queue = queue.Queue()

async handle_ws_msg():
    pass

def handle_ros_thread():
    rclpy.init()
    node = rclpy.create_node('ws_server')
    pub = node.create_publisher(String, 'ws_scan3dlive', 10)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
    node.destroy_node()
    pub.destroy()

def handle_ws_thread():
    pass


with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
    ros_future = executor.submit(handle_ros_thread)
    ws_future = executor.submit(handle_ws_thread)
    concurrent.futures.wait([ros_future, ws_future])

