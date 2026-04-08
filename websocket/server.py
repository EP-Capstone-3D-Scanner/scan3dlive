import rclpy 
from rclpy.node import Node 
from rclpy.qos import qos_profile_sensor_data 
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import asyncio 
import queue 
import concurrent.futures 
import websockets
import ssl
import pathlib
import json
import subprocess
import time
import os
import signal

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
        print("Scan3DLiveNode initialized, subscribing to /cloud_registered") 
        self.sub = self.create_subscription(
            PointCloud2, 
            '/cloud_registered', 
            self.callback, 
            qos_profile_sensor_data
        ) 
    
    def callback(self, msg):
        start_time = time.perf_counter()
        #print("ran")
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
        #print(f"Execution time: {elapsed_time:.4f} seconds")


async def handler(websocket): 
    print(f"Client connected from {websocket.remote_address}")
    client_ready = asyncio.Event()
    client_ready.set()
    
    scanner_processes = []
    scanner_commands = [
        "cd ../livox_ros2_driver_ext_ws/ && source install/setup.bash && ros2 launch livox_ros2_driver_ext_test lidar_launch.py",
        "cd ../livox_ros2_driver_ext_ws/ && source install/setup.bash && ros2 run livox_ros2_driver_ext_test lidar_formatter",
        "cd ../usb_cam_ws/ && source install/setup.bash && ros2 launch usb_cam_test camera.launch.py",
        "cd ../usb_cam_ws/ && source install/setup.bash && ros2 run usb_cam_formatter formatter",
        "cd ../fast-livo2_ws && source install/setup.bash && ros2 launch fast_livo2_test launch.py"
    ]



    MAX_PAYLOAD_BYTES = 512 * 1024  # 1024 KB target max size (tune as needed)

    async def send_data_loop():
        while True:
            await client_ready.wait() # Block until UI sends an ACK
            
            try:
                # Wait until there is at least ONE message in the queue
                while ws_msg_queue.empty():
                    await asyncio.sleep(0.01)
                    
                payload = bytearray()
                
                # Drain the queue to build a larger batched chunk
                while not ws_msg_queue.empty():
                    try:
                        msg = ws_msg_queue.get_nowait()
                        payload.extend(msg)
                        
                        # Stop pulling from queue if we reach our target size
                        # (It's okay if the final message pushes it slightly over the limit)
                        if len(payload) >= MAX_PAYLOAD_BYTES:
                            break
                    except queue.Empty:
                        break
                
                # Send the batched payload
                if payload and len(payload)>0:
                    print(len(payload))
                    await websocket.send(payload)
                    client_ready.clear() # Block until the next ACK
                    
            except websockets.exceptions.ConnectionClosed:
                break

    async def receive_command_loop():
        nonlocal scanner_processes
        nonlocal scanner_commands

        try:
            async for message in websocket:
                print(ws_msg_queue.qsize())
                if isinstance(message, str): # Commands and ACKs will be text/JSON
                    try:
                        data = json.loads(message)
                        msg_type = data.get("type")
                        
                        if msg_type == "ACK":
                            client_ready.set()

                        elif msg_type == "COMMAND":
                            action = data.get("action")
                            
                            if action == "start_scanner":
                                # Check if processes are already running (list is not empty)
                                if not scanner_processes:
                                    print("Starting scanner scripts...")
                                    
                                    # Start the first 4 commands
                                    for i in range(4):
                                        print(f"Starting process {i+1}/5...")
                                        proc = subprocess.Popen(
                                            scanner_commands[i], 
                                            shell=True, 
                                            executable='/bin/bash',
                                            stdout=subprocess.DEVNULL, 
                                            stderr=subprocess.DEVNULL,
                                            text=True,
                                            preexec_fn=os.setsid # Groups processes so ROS launches can be killed cleanly
                                        )
                                        scanner_processes.append(proc)
                                    
                                    # Wait a brief moment to let the first 4 initialize
                                    print("Waiting for initial ROS nodes to spin up...")
                                    time.sleep(3)
                                    
                                    # Start the 5th command (fast_livo2)
                                    print("Starting process 5/5 (fast_livo2)...")
                                    proc = subprocess.Popen(
                                        scanner_commands[4], 
                                        shell=True, 
                                        executable='/bin/bash',
                                        stdout= subprocess.DEVNULL, #None
                                        stderr= subprocess.DEVNULL, #None
                                        text=True,
                                        preexec_fn=os.setsid
                                    )
                                    scanner_processes.append(proc)
                                    
                            elif action == "stop_scanner":
                                if scanner_processes:
                                    print("Stopping all scanner scripts...")
                                    for proc in scanner_processes:
                                        # Kill the entire process group to ensure ROS child nodes die
                                        if proc.poll() is None:
                                            try:
                                                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                                                proc.wait(timeout=2)
                                            except Exception as e:
                                                print(f"Error terminating process: {e}")
                                    
                                    # Clear the list so they can be started again
                                    scanner_processes.clear()
                                    print("All scanner processes stopped.")

                    except json.JSONDecodeError:
                        print("Received malformed JSON")
        except websockets.exceptions.ConnectionClosed:
            pass
        
        finally:
            # Cleanup processes if the client unexpectedly disconnects
            if scanner_processes:
                print("Client disconnected. Cleaning up scanner processes...")
                for proc in scanner_processes:
                    if proc.poll() is None:
                        try:
                            # Use killpg to ensure the shell and all child ROS nodes are terminated
                            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                            proc.wait(timeout=2)
                        except Exception as e:
                            print(f"Error terminating process during cleanup: {e}")
                
                # Clear the list so it's fresh for the next connection
                scanner_processes.clear()

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

async def ws_main():
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    cert_path = pathlib.Path(__file__).with_name("localhost.pem")
    ssl_context.load_cert_chain(cert_path) 
    
    print("Starting secure websocket (WSS) handler thread on wss://0.0.0.0:8001...") 
    
    async with websockets.serve(handler, '0.0.0.0', 8001, ssl=ssl_context, max_size= 2 * 1024 * 1024) as server: 
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
    
