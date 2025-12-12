#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import sensor_msgs.msg
import std_msgs.msg
from sensor_msgs_py import point_cloud2
# import cv_bridge  # REMOVED

import socket
import iphone
import ctypes
import numpy as np
import cv2
import threading
import queue
import PIL
import io
import scipy.spatial.transform
import json

def receive(sock, message_size):
     chunks = []
     bytes_recd = 0
     while bytes_recd < message_size:
          chunk = sock.recv(min(message_size - bytes_recd, 2048)) # Receive in chunks
          if not chunk:
               raise RuntimeError("Socket connection broken")
          chunks.append(chunk)
          bytes_recd += len(chunk)
     return b''.join(chunks)

def cv2_to_imgmsg(cv_image, encoding="bgr8"):
    """Convert OpenCV image to ROS Image message without cv_bridge"""
    img_msg = sensor_msgs.msg.Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = encoding
    
    if encoding == "bgr8":
        img_msg.step = cv_image.shape[1] * 3
    elif encoding == "mono8":
        img_msg.step = cv_image.shape[1]
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")
    
    img_msg.data = cv_image.tobytes()
    img_msg.is_bigendian = 0
    
    return img_msg

class StandaloneDriver(Node):

    def __init__(self):
        super().__init__('standalone_driver')
        self.declare_parameter('port',8888)

        # iPhone depth intrinsics
        self.declare_parameter('depth_max_distance', 10.0)  # Maximum distance in meters for mapping
        self.declare_parameter('depth_downsample_factor', 10)
        self.declare_parameter('focal_length_x', 256.0)
        self.declare_parameter('focal_length_y', 192.0)
        self.declare_parameter('principal_point_x', 128.0)
        self.declare_parameter('principal_point_y', 96.0)

        self.run_ = True
        # self.bridge = cv_bridge.CvBridge()  # REMOVED
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
        eo_info_pub = None
        depth_pub = None
        depth_info_pub = None
        imu_pub = None
        mag_pub = None
        iphone_name = ""
        msg_queue = queue.Queue()
        max_dist = float(self.get_parameter('depth_max_distance').get_parameter_value().double_value)

        def worker():
            nonlocal msg_queue
            nonlocal iphone_name
            nonlocal eo_pub
            nonlocal eo_info_pub
            nonlocal depth_pub
            nonlocal depth_info_pub
            nonlocal imu_pub
            nonlocal mag_pub

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
                        eo_pub = self.create_publisher(sensor_msgs.msg.Image,f"/iphone/{iphone_name}/eo/raw",10)
                        eo_info_pub = self.create_publisher(sensor_msgs.msg.CameraInfo,f"/iphone/{iphone_name}/eo/camera_info",10)
                        depth_pub = self.create_publisher(sensor_msgs.msg.Image,f"/iphone/{iphone_name}/depth/raw",10)
                        depth_info_pub = self.create_publisher(sensor_msgs.msg.CameraInfo,f"/iphone/{iphone_name}/depth/camera_info",10)
                        imu_pub = self.create_publisher(sensor_msgs.msg.Imu,f"/iphone/{iphone_name}/imu",10)
                        mag_pub = self.create_publisher(sensor_msgs.msg.MagneticField,f"/iphone/{iphone_name}/mag",10)
                        self.get_logger().info(f"Name for {addr}: {iphone_name}")
                    case iphone.MessageIDs.CameraFrame:
                        im_np = np.frombuffer(data,np.uint8)
                        im_opencv = cv2.imdecode(im_np, cv2.IMREAD_COLOR)
                        im_ros = cv2_to_imgmsg(im_opencv, encoding="bgr8")
                        im_ros.header.stamp.sec = header.timestamp_sec
                        im_ros.header.stamp.nanosec = header.timestamp_nsec
                        #self.get_logger().info(f"{header.timestamp_nsec*1e-9}")
                        ci += 1
                        if eo_pub:
                            eo_pub.publish(im_ros)
                    case iphone.MessageIDs.DepthFrame:
                        # Read depth data as float32
                        # Clip values to max distance and normalize to 0-255
                        # Closer distances map to higher values (brighter)
                        # Read depth data as float32 (192x256 depth map)
                        depth_map = np.frombuffer(data, np.float32).reshape((192, 256))
                        
                        # Publish raw depth map as 32FC1 Image
                        depth_image = sensor_msgs.msg.Image()
                        depth_image.header.stamp.sec = header.timestamp_sec
                        depth_image.header.stamp.nanosec = header.timestamp_nsec
                        depth_image.header.frame_id = "camera_depth_optical_frame"
                        depth_image.height = 192
                        depth_image.width = 256
                        depth_image.encoding = "32FC1"  # 32-bit float, 1 channel
                        depth_image.is_bigendian = 0
                        depth_image.step = 256 * 4  # 256 pixels * 4 bytes per float32
                        depth_image.data = depth_map.tobytes()
                        depth_image.header.stamp.sec = header.timestamp_sec
                        depth_image.header.stamp.nanosec = header.timestamp_nsec

                        li += 1
                        if depth_pub:
                            depth_pub.publish(depth_image)
                    case iphone.MessageIDs.CameraInfo:
                        msg = json.loads(data.decode('utf-8'))
                        # Reshape the matrix
                        K = np.array(msg['camera_matrix']).reshape((3,3)).T
                        self.get_logger().info(f"Cam matrix: {K}")

                        eo_info = sensor_msgs.msg.CameraInfo()

                        eo_info.distortion_model = 'pinhole'
                        eo_info.height = msg['image_height']
                        eo_info.width = msg['image_width']
                        eo_info.k = K.flatten()
                        eo_info.r = [1,0,0,0,1,0,0,0,1]
                        cam_spacing = 140 # mm
                        eo_info.p = [
                            *K[0,:], 0,
                            *K[1,:], -K[1,1] * cam_spacing,
                            *K[2,:], 0
                        ]

                        sx = 256 / msg['image_width']
                        sy = 192 / msg['image_height']

                        K[0,:] *= sx
                        K[1,:] *= sy

                        depth_info = sensor_msgs.msg.CameraInfo()

                        depth_info.distortion_model = 'pinhole'
                        depth_info.height = 192
                        depth_info.width = 256
                        depth_info.k = K.flatten()
                        depth_info.r = [1,0,0,0,1,0,0,0,1]
                        cam_spacing = 140 # mm
                        depth_info.p = [
                            *K[0,:], 0,
                            *K[1,:], -K[1,1] * cam_spacing,
                            *K[2,:], 0
                        ]

                        depth_info.header.stamp.sec = eo_info.header.stamp.sec = header.timestamp_sec
                        depth_info.header.stamp.nanosec = eo_info.header.stamp.nanosec = header.timestamp_nsec

                        if eo_info_pub:
                            eo_info_pub.publish(eo_info)
                        if depth_info_pub:
                            depth_info_pub.publish(depth_info)

                    case iphone.MessageIDs.GPSMessage:
                        pass
                    case iphone.MessageIDs.IMUMessage:
                        imui = iphone.IMU.from_buffer_copy( data )

                        imu = sensor_msgs.msg.Imu()
                        mag = sensor_msgs.msg.MagneticField()

                        q = scipy.spatial.transform.Rotation.from_euler('zyx',[imui.yaw,imui.pitch,imui.roll],degrees=False).as_quat()
                        imu.orientation.x = q[0]
                        imu.orientation.y = q[1]
                        imu.orientation.z = q[2]
                        imu.orientation.w = q[3]
                        imu.angular_velocity.x = imui.rotx
                        imu.angular_velocity.y = imui.roty
                        imu.angular_velocity.z = imui.rotz
                        imu.linear_acceleration.x = ( imui.gx + imui.accx ) * 9.80665
                        imu.linear_acceleration.y = ( imui.gy + imui.accy ) * 9.80665
                        imu.linear_acceleration.z = ( imui.gz + imui.accz ) * 9.80665
                        mag.magnetic_field.x = imui.magx * 1e-6
                        mag.magnetic_field.y = imui.magy * 1e-6
                        mag.magnetic_field.z = imui.magz * 1e-6

                        imu.header.frame_id = "world"
                        mag.header.frame_id = "world"

                        mag.header.stamp.sec = imu.header.stamp.sec = header.timestamp_sec
                        mag.header.stamp.nanosec = imu.header.stamp.nanosec = header.timestamp_nsec

                        if imu_pub:
                            imu_pub.publish(imu)
                        if mag_pub:
                            mag_pub.publish(mag)

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
