#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node

import sensor_msgs.msg
import cv_bridge

import asyncio
import utm
import datetime
import traceback
import socket
import iphone
import ctypes
import numpy as np
import cv2
import threading
import queue

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
        #self.publisher_ = self.create_publisher(Customgps, '/gps', 10)
        self.run_ = True
        self.bridge = cv_bridge.CvBridge()
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        port = int(self.get_parameter('port').get_parameter_value().integer_value)

        self.get_logger().info('HERE 0')
        tasks = []
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as self.socket:
            self.socket.bind(('0.0.0.0',port))
            self.socket.listen()
            self.get_logger().info(f"Listening for topics on port {port}")
            while self.run_:
                tasks.append(threading.Thread(target=self.run_single,args=self.socket.accept()))
                tasks[-1].start()
        for task in task:
            task.join()

    def run_single(self, connection, addr):
        pub = None
        iphone_name = ""
        msg_queue = queue.Queue()

        def worker():
            self.get_logger().info(f"In worker!")
            nonlocal pub
            nonlocal iphone_name
            nonlocal msg_queue
            while True:
                task = msg_queue.get()
                if task is None:
                    break

                header, data = task

                match header.message_id:
                    case iphone.MessageIDs.iPhoneName:
                        iphone_name = data.decode('utf-8')
                        pub = self.create_publisher(sensor_msgs.msg.Image,f"/im_{iphone_name}",10)
                        self.get_logger().info(f"Name: {data}")
                    case iphone.MessageIDs.CameraFrame:
                        im_np = np.frombuffer(data,np.uint8)
                        im_opencv = cv2.imdecode(im_np, cv2.IMREAD_COLOR)
                        im_ros = self.bridge.cv2_to_imgmsg(im_opencv, encoding="bgr8")
                        im_ros.header.stamp.sec = header.timestamp_sec
                        im_ros.header.stamp.nanosec = header.timestamp_nsec
                        if pub:
                            self.get_logger().info(f"Publishing...")
                            pub.publish(im_ros)

                msg_queue.task_done()

        workers = []
        for i in range(12):
            workers.append(threading.Thread(target=worker))
            workers[-1].start()

        try:
            while self.run_:
                header = iphone.Header.from_buffer_copy( receive( connection, ctypes.sizeof( iphone.Header ) ) )
                if header.payload_length:
                    data = receive( connection, header.payload_length )
                else:
                    data = None
                msg_queue.put((header,data))
        except RuntimeError:
            pass

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
