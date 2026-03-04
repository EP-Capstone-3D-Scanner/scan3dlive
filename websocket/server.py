import rclpy 
from rclpy.node import Node 
from rclpy.qos import qos_profile_sensor_data 
from livox_interfaces.msg import CustomMsg
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import asyncio 
import queue 
import concurrent.futures 
import websockets
import struct
import ssl
import pathlib

"""
ros2 topic pub /livox/lidar livox_interfaces/msg/CustomMsg "{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'livox_frame'}, timebase: 1000000000, point_num: 2, lidar_id: 1, rsvd: [0,0,0], points: [{offset_time: 0, x: 1.0, y: 0.0, z: 0.5, reflectivity: 120, tag: 0, line: 1}, {offset_time: 1000, x: 1.2, y: 0.1, z: 0.6, reflectivity: 100, tag: 0, line: 2}] }"
"""

ws_msg_queue = queue.Queue()
BYTES_PER_POINT = 16

class Scan3DLiveNode(Node): 
    def __init__(self): 
        super().__init__('scan3dliveui') 
        print("Scan3DLiveNode initialized, subscribing to /livox/lidar") 
        self.sub = self.create_subscription(PointCloud2, '/cloud_registered', self.callback, qos_profile_sensor_data) 
    
    def callback(self, msg):
        ba = bytearray()
        points = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        print(points)
        for point in points:
            s = struct.pack('>f', point[3])
            i = struct.unpack('>l', s)[0]
            r = (i & 0x00FF0000) >> 16
            g = (i & 0x0000FF00) >> 8
            b = (i & 0x000000FF) >> 0
            ba.extend(struct.pack('<fffBBBB', -point[1], point[2]+0.5, -point[0], r, g, b, 255))
        ws_msg_queue.put_nowait(ba) 

# WebSocket server to send messages from ROS to clients 
async def handler(websocket): 
    print(f"Client connected from {websocket.remote_address}")
    while True: 
        try:
            # Use get_nowait() to prevent blocking the asyncio event loop!
            msg = ws_msg_queue.get_nowait() 
            await websocket.send(msg) 
        except queue.Empty:
            # Yield control back to the event loop if no messages are ready
            await asyncio.sleep(0.01)
        except websockets.exceptions.ConnectionClosed:
            print("Client disconnected.")
            break

async def ws_main():
    # Set up the SSL Context for WSS
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    
    # Load the certificate and private key
    # Make sure this PEM file contains BOTH the private key and the certificate!
    cert_path = pathlib.Path(__file__).with_name("localhost.pem")
    ssl_context.load_cert_chain(cert_path) 
    
    print("Starting secure websocket (WSS) handler thread on wss://0.0.0.0:8001...") 
    
    # Passing the ssl_context to serve() makes it a wss:// connection
    async with websockets.serve(handler, '0.0.0.0', 8001, ssl=ssl_context) as server: 
        await server.serve_forever() 

def handle_ws_thread(): 
    # Create a new event loop for this specific thread
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
    
