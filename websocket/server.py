from geometry_msgs.msg import PointCloud2
import rclpy
import rclpy.executors
import rclpy.serialization
import asyncio
import queue
import concurrent.futures
import websockets

ws_msg_queue = queue.Queue()

# ROS node to send points from /cloud_registered topic to WebSocket clients
class Scan3DLiveNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('scan3d_live_node')
        self.sub = self.create_subscription(PointCloud2, '/cloud_registered', self.callback, 10)

    def callback(self, msg):
        ws_msg_queue.put(rclpy.serialization.serialize_message(msg))

def handle_ros_thread():
    rclpy.init()
    node = Scan3DLiveNode()
    executor = rclpy.executors.SingleThreadedExecutor() # Don't necessarily need executor for a single node, but it allows for future expansion
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
    node.destroy_node()

# WebSocket server to send messages from ROS to clients
async def send_ws_msg(websocket, path):
    while True:
        msg = ws_msg_queue.get()
        if(msg is not None):
            await websocket.send(msg)
        asyncio.sleep(0.01)  # small delay to prevent busy waiting

async def ws_thread_main():
    async with websockets.serve(send_ws_msg, 'localhost', 8001) as server:
        await server.serve_forever()

def handle_ws_thread():
    asyncio.run(ws_thread_main())

if __name__ == '__main__':
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        ros_future = executor.submit(handle_ros_thread)
        ws_future = executor.submit(handle_ws_thread)
        concurrent.futures.wait([ros_future, ws_future])

