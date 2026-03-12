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
import json
import subprocess

"""
ros2 topic pub /livox/lidar livox_interfaces/msg/CustomMsg "{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'livox_frame'}, timebase: 1000000000, point_num: 2, lidar_id: 1, rsvd: [0,0,0], points: [{offset_time: 0, x: 1.0, y: 0.0, z: 0.5, reflectivity: 120, tag: 0, line: 1}, {offset_time: 1000, x: 1.2, y: 0.1, z: 0.6, reflectivity: 100, tag: 0, line: 2}] }"
"""

ws_msg_queue = queue.Queue()
BYTES_PER_POINT = 16
import time
import numpy as np

class Scan3DLiveNode(Node): 
    def __init__(self): 
        super().__init__('scan3dliveui') 
        print("Scan3DLiveNode initialized, subscribing to /livox/lidar") 
        self.sub = self.create_subscription(
            PointCloud2, 
            '/cloud_registered', 
            self.callback, 
            qos_profile_sensor_data
        ) 
    
    def callback(self, msg):
        start_time = time.perf_counter()
        
        try:
            points = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
            
            if len(points) > 0:
                
                x = points['x']
                y = points['y']
                z = points['z']
                rgb_float = points['rgb']
                rgb_int = rgb_float.view(np.uint32)
                
                r = (rgb_int & 0x00FF0000) >> 16
                g = (rgb_int & 0x0000FF00) >> 8
                b = (rgb_int & 0x000000FF)
                
                dt = np.dtype([
                    ('x', '<f4'),
                    ('y', '<f4'),
                    ('z', '<f4'),
                    ('r', 'u1'),
                    ('g', 'u1'),
                    ('b', 'u1'),
                    ('a', 'u1')
                ])
                
                out_arr = np.empty(len(points), dtype=dt)
                out_arr['x'] = -y
                out_arr['y'] = z + 0.5
                out_arr['z'] = -x
                out_arr['r'] = r
                out_arr['g'] = g
                out_arr['b'] = b
                out_arr['a'] = 255
                
                ba = bytearray(out_arr.tobytes())
                ws_msg_queue.put_nowait(ba) 
                
        except Exception as e:
            print(f"CRITICAL ERROR in callback: {e}")
            return
            
        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        print(f"Execution time: {elapsed_time:.4f} seconds")


async def handler(websocket): 
    print(f"Client connected from {websocket.remote_address}")
    
    # Flow control event: True when the client is ready for data
    client_ready = asyncio.Event()
    client_ready.set() # Assume ready for the very first message
    
    scanner_process = None

    async def send_data_loop():
        while True:
            await client_ready.wait() # Block until UI sends an ACK
            try:
                msg = ws_msg_queue.get_nowait() 
                await websocket.send(msg)
                client_ready.clear() # Clear the flag to block until the next ACK
            except queue.Empty:
                await asyncio.sleep(0.01)
            except websockets.exceptions.ConnectionClosed:
                break

    async def receive_command_loop():
        nonlocal scanner_process
        try:
            async for message in websocket:
                if isinstance(message, str): # Commands and ACKs will be text/JSON
                    try:
                        data = json.loads(message)
                        msg_type = data.get("type")
                        
                        if msg_type == "ACK":
                            client_ready.set() # Unblock the send_data_loop
                            
                        elif msg_type == "COMMAND":
                            action = data.get("action")
                            if action == "start_scanner":
                                if scanner_process is None or scanner_process.poll() is not None:
                                    print("Starting scanner script...")
                                    #command = "source /opt/ros/humble/setup.bash && ros2 bag play ../datasets/rosbag2_2026_03_12-00_08_30"
                                    #command = "source /opt/ros/humble/setup.bash && ros2 bag play ../../rosbag2_2026_03_12-00_08_30_0.db3"
                                    command = "source /opt/ros/humble/setup.bash && ros2 bag play ../../rosbag2_2026_03_06-22_57_43_0.db3"
                                    scanner_process = subprocess.Popen(command, shell=True, executable='/bin/bash',stdout=subprocess.PIPE, stderr=subprocess.PIPE,text=True)
                                    #scanner_process = subprocess.Popen(["bash", "-c", "touch hi.txt"])
                                    #scanner_process = subprocess.Popen("./start_scanner.sh > hi.txt", shell=True)
                            elif action == "stop_scanner":
                                if scanner_process is not None and scanner_process.poll() is None:
                                    print("Stopping scanner script...")
                                    scanner_process.terminate()
                                    scanner_process.wait()
                                    scanner_process = None
                    except json.JSONDecodeError:
                        print("Received malformed JSON")
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            # Cleanup process if the client unexpectedly disconnects
            if scanner_process is not None and scanner_process.poll() is None:
                scanner_process.terminate()

    # Run both loops concurrently
    send_task = asyncio.create_task(send_data_loop())
    recv_task = asyncio.create_task(receive_command_loop())
    
    # Wait for either to finish (which happens on disconnect)
    done, pending = await asyncio.wait(
        [send_task, recv_task], 
        return_when=asyncio.FIRST_COMPLETED
    )
    
    # Cancel whichever loop is still lingering
    for task in pending:
        task.cancel()
        
    print("Client disconnected.")

"""
async def handler(websocket): 
    print(f"Client connected from {websocket.remote_address}")
    while True: 
        try:
            msg = ws_msg_queue.get_nowait() 
            await websocket.send(msg)
            await asyncio.sleep(0.01) 
        except queue.Empty:
            await asyncio.sleep(0.01)
        except websockets.exceptions.ConnectionClosed:
            print("Client disconnected.")
            break
"""

async def ws_main():
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    cert_path = pathlib.Path(__file__).with_name("localhost.pem")
    ssl_context.load_cert_chain(cert_path) 
    
    print("Starting secure websocket (WSS) handler thread on wss://0.0.0.0:8001...") 
    
    async with websockets.serve(handler, '0.0.0.0', 8001, ssl=ssl_context) as server: 
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
    
