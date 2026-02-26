import rclpy 
from rclpy.node import Node 
from rclpy.qos import qos_profile_sensor_data 
from livox_interfaces.msg import CustomMsg
import rclpy.serialization 
import asyncio 
import queue 
import concurrent.futures 
import websockets 
import struct

"""
ros2 topic pub /livox/lidar livox_interfaces/msg/CustomMsg "{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'livox_frame'}, timebase: 1000000000, point_num: 2, lidar_id: 1, rsvd: [0,0,0], points: [{offset_time: 0, x: 1.0, y: 0.0, z: 0.5, reflectivity: 120, tag: 0, line: 1}, {offset_time: 1000, x: 1.2, y: 0.1, z: 0.6, reflectivity: 100, tag: 0, line: 2}] }"
"""

# Make sure to run source livox_ros2_driver install/setup.bash before running this script to ensure ROS can find the custom messages. You may also need to run colcon build in the workspace after adding the new message definitions.
ws_msg_queue = queue.Queue()
BYTES_PER_POINT = 12

class Scan3DLiveNode(Node): 
    def __init__(self): 
        super().__init__('scan3dliveui') 
        print("Scan3DLiveNode initialized, subscribing to /livox/lidar") 
        self.sub = self.create_subscription(CustomMsg, '/livox/lidar', self.callback, qos_profile_sensor_data) 
    
    def callback(self, msg):
        b = bytearray()
        for point in msg._points:
            b.extend(struct.pack('<fff', point.x, point.y, point.z))
        ws_msg_queue.put_nowait(b) 

# WebSocket server to send messages from ROS to clients 
async def handler(websocket): 
    while True: 
        msg = ws_msg_queue.get() 
        if msg: 
            await websocket.send(msg) 

async def ws_main(): 
    print("Starting websocket handler thread...") 
    async with websockets.serve(handler, '0.0.0.0', 8001) as server: 
        await server.serve_forever() 

def handle_ws_thread(): 
    asyncio.run(ws_main()) 

def handle_ros_thread():
    print("Starting ros2 handler thread...") 
    rclpy.init() 
    node = Scan3DLiveNode() 
    rclpy.spin(node)
    rclpy.shutdown() 
    node.destroy_node() 
    
if __name__ == '__main__': 
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor: 
        ros_future = executor.submit(handle_ros_thread) 
        ws_future = executor.submit(handle_ws_thread) 
        concurrent.futures.wait([ros_future, ws_future])
    
