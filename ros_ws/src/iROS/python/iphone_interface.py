#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import sensor_msgs.msg
import cv_bridge

import socket
import iphone
import ctypes
import numpy as np
import cv2
import threading
import queue
import PIL
import io

def receive(sock, message_size):
     chunks = []
     bytes_recd = 0
     while bytes_recd < message_size:
          chunk = sock.recv(min(message_size - bytes_recd, 2048)) # Receive in chunks
          print(chunk)
          if not chunk:
               raise RuntimeError("Socket connection broken")
          chunks.append(chunk)
          bytes_recd += len(chunk)
     return b''.join(chunks)

class StandaloneDriver(Node):

    def __init__(self):
        super().__init__('standalone_driver')
        self.declare_parameter('port',8888)
        self.declare_parameter('lidar_max_distance', 10.0)  # Maximum distance in meters for mapping
        self.run_ = True
        self.bridge = cv_bridge.CvBridge()
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        port = int(self.get_parameter('port').get_parameter_value().integer_value)

        tasks = []
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as self.socket:
            self.socket.bind(('0.0.0.0',port))
            self.socket.listen()
            self.get_logger().info(f"Listening for topics on port {port}")
            while self.run_:
                tasks.append(threading.Thread(target=self.run_single,args=self.socket.accept()))
                tasks[-1].start()
        for task in tasks:
            task.join()

    def run_single(self, connection, addr):
        self.get_logger().info(f"Connection made, remote addr: {addr}")
        eo_pub = None
        lidar_pub = None
        iphone_name = ""
        msg_queue = queue.Queue()
        max_dist = float(self.get_parameter('lidar_max_distance').get_parameter_value().double_value)

        def worker():
            nonlocal eo_pub
            nonlocal lidar_pub
            nonlocal iphone_name
            nonlocal msg_queue

            ci = 0
            li = 0

            while True:
                task = msg_queue.get()
                if task is None:
                    break

                header, data = task

                match header.message_id:
                    case iphone.MessageIDs.iPhoneName:
                        iphone_name = data.decode('utf-8')
                        eo_pub = self.create_publisher(sensor_msgs.msg.Image,f"/iphone/{iphone_name}/eo",10)
                        lidar_pub = self.create_publisher(sensor_msgs.msg.Image,f"/iphone/{iphone_name}/lidar",10)
                        self.get_logger().info(f"Name for {addr}: {iphone_name}")
                    case iphone.MessageIDs.CameraFrame:
                        im_np = np.frombuffer(data,np.uint8)
                        im_opencv = cv2.imdecode(im_np, cv2.IMREAD_COLOR)
                        im_ros = self.bridge.cv2_to_imgmsg(im_opencv, encoding="bgr8")
                        im_ros.header.stamp.sec = header.timestamp_sec
                        im_ros.header.stamp.nanosec = header.timestamp_nsec
                        #self.get_logger().info(f"{header.timestamp_nsec*1e-9}")
                        ci += 1
                        if eo_pub:
                            eo_pub.publish(im_ros)
                    case iphone.MessageIDs.LidarFrame:
                        # Read depth data as float32
                        # Clip values to max distance and normalize to 0-255
                        # Closer distances map to higher values (brighter)
                        print(len(data))
                        depth_clipped = np.clip(np.frombuffer(data, np.float32).reshape((192, 256)), 0, max_dist)
                        grayscale = ((max_dist - depth_clipped) / max_dist * 255).astype(np.uint8)
                        # Convert to ROS image message as mono8
                        im_ros = self.bridge.cv2_to_imgmsg(grayscale, encoding="mono8")
                        im_ros.header.stamp.sec = header.timestamp_sec
                        im_ros.header.stamp.nanosec = header.timestamp_nsec
                        li += 1
                        if eo_pub:
                            lidar_pub.publish(im_ros)
                    case iphone.MessageIDs.GPSMessage:
                        pass
                    case iphone.MessageIDs.IMUMessage:
                        pass

                msg_queue.task_done()

        workers = []
        for i in range(12):
            workers.append(threading.Thread(target=worker))
            workers[-1].start()

        try:
            while self.run_:
                header = iphone.Header.from_buffer_copy( receive( connection, ctypes.sizeof( iphone.Header ) ) )
                self.get_logger().info(f"{header.timestamp_nsec*1e-9}")
                if header.payload_length:
                    data = receive( connection, header.payload_length )
                else:
                    data = None
                msg_queue.put((header,data))
        except RuntimeError:
            pass

        self.get_logger().info(f"Connection to {iphone_name} {addr} closed!")

        # Send signal to all workers to stop
        for i in range(12):
            msg_queue.put(None)
        
        for w in workers:
            w.join()

    def on_shutdown(self):
        self.run_ = False
        self.socket.close()
        self.thread.join()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = StandaloneDriver()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.on_shutdown()
        minimal_publisher.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
