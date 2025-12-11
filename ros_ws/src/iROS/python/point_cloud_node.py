#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
import std_msgs.msg
import numpy as np
import re
import threading

class PointCloudNode(Node):
    def __init__(self):
        super().__init__('pointcloud_node')
        
        # Declare parameters
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('downsample_factor', 10)
        self.declare_parameter('focal_length_x', 256.0)
        self.declare_parameter('focal_length_y', 192.0)
        self.declare_parameter('principal_point_x', 128.0)
        self.declare_parameter('principal_point_y', 96.0)
        
        # Get parameters
        self.max_depth = self.get_parameter('max_depth').value
        self.downsample_factor = self.get_parameter('downsample_factor').value
        self.focal_length_x = self.get_parameter('focal_length_x').value
        self.focal_length_y = self.get_parameter('focal_length_y').value
        self.center_x = self.get_parameter('principal_point_x').value
        self.center_y = self.get_parameter('principal_point_y').value
        
        # Track subscriptions and publishers
        self.lidar_subscriptions = {}
        self.pointcloud_publishers = {}
        
        self.get_logger().info('PointCloud Node ready (auto-discovery mode)')
        self.get_logger().info(f'  Max depth: {self.max_depth}m')
        self.get_logger().info(f'  Downsample: 1/{self.downsample_factor}')
        self.get_logger().info(f'  Focal length: focal_length_x={self.focal_length_x}, focal_length_y={self.focal_length_y}')
        self.get_logger().info(f'  Principal point: center_x={self.center_x}, center_y={self.center_y}')
        
        # Start topic discovery
        self.check_topics()
    
    def check_topics(self):
        """Automatically discover lidar depth topics and create pointcloud publishers"""
        for node_name in self.get_node_names():
            for topic_name, topic_types in self.get_publisher_names_and_types_by_node(node_name, ''):
                # Match pattern: /iphone/{device_name}/lidar (raw depth map)
                if re.match(r"^/iphone/.*/lidar$", topic_name):
                    if topic_name not in self.lidar_subscriptions:
                        self.get_logger().info(f"Discovered lidar depth topic: {topic_name}")
                        self.initialize_converter(topic_name)
        
        # Check again in 1 second
        self.timer = threading.Timer(1, self.check_topics).start()
    
    def initialize_converter(self, lidar_topic):
        """Initialize subscription and publisher for a lidar depth topic"""
        # Create corresponding pointcloud topic name
        pointcloud_topic = lidar_topic.replace('/lidar', '/pointcloud')
        
        # Create subscriber for lidar depth images
        self.lidar_subscriptions[lidar_topic] = self.create_subscription(
            Image,
            lidar_topic,
            lambda msg, topic=pointcloud_topic: self.lidar_callback(topic, msg),
            10
        )
        
        # Create publisher for point clouds
        self.pointcloud_publishers[pointcloud_topic] = self.create_publisher(
            PointCloud2,
            pointcloud_topic,
            10
        )
        
        self.get_logger().info(f"Created converter: {lidar_topic} -> {pointcloud_topic}")
    
    @staticmethod
    def depth_map_to_pointcloud(depth_map, max_depth, downsample_factor, 
                                focal_length_x, focal_length_y,
                                principal_point_x, principal_point_y):
        """Convert depth map to 3D point cloud with filtering and downsampling"""
        height, width = depth_map.shape
        point_cloud_data = []
        
        # Downsample at source - only process every Nth pixel
        for pixel_row in range(0, height, downsample_factor):
            for pixel_col in range(0, width, downsample_factor):
                depth_value = depth_map[pixel_row, pixel_col]
                
                # Filter: valid depth within range
                if depth_value > 0 and depth_value < max_depth:
                    # Unproject to 3D using pinhole camera model
                    point_x = (pixel_col - principal_point_x) * depth_value / focal_length_x
                    point_y = (pixel_row - principal_point_y) * depth_value / focal_length_y
                    point_z = depth_value
                    point_cloud_data.append([point_x, point_y, point_z])
        
        return np.array(point_cloud_data, dtype=np.float32) if point_cloud_data else np.array([])
    
    @staticmethod
    def create_pointcloud2_message(points, header_stamp_sec, header_stamp_nsec, frame_id="camera_depth_optical_frame"):
        """Create PointCloud2 message from numpy array of points"""
        if len(points) == 0:
            return None
        
        # Create header
        pointcloud_header = std_msgs.msg.Header()
        pointcloud_header.stamp.sec = header_stamp_sec
        pointcloud_header.stamp.nanosec = header_stamp_nsec
        pointcloud_header.frame_id = frame_id
        
        # Create PointCloud2 message
        pointcloud_msg = point_cloud2.create_cloud_xyz32(pointcloud_header, points)
        
        return pointcloud_msg
    
    def lidar_callback(self, pointcloud_topic, msg):
        """Process lidar depth image and publish point cloud"""
        try:
            device_name = pointcloud_topic.split('/')[2]
            
            # Validate encoding
            if msg.encoding != "32FC1":
                self.get_logger().error(f'[{device_name}] Unexpected encoding: {msg.encoding}')
                return
            
            # Convert ROS Image to numpy array
            depth_map = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            
            # Convert to point cloud
            points = PointCloudNode.depth_map_to_pointcloud(
                depth_map,
                self.max_depth,
                self.downsample_factor,
                self.focal_length_x,
                self.focal_length_y,
                self.center_x,
                self.center_y
            )
            
            if len(points) == 0:
                self.get_logger().warn(f'[{device_name}] No valid points in depth map')
                return
            
            # Create PointCloud2 message
            pointcloud_msg = PointCloudNode.create_pointcloud2_message(
                points,
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                msg.header.frame_id
            )
            
            if pointcloud_msg:
                self.pointcloud_publishers[pointcloud_topic].publish(pointcloud_msg)
                self.get_logger().debug(f'[{device_name}] Published {len(points)} points')
        
        except Exception as e:
            self.get_logger().error(f'PointCloud conversion error: {e}')


def main():
    rclpy.init()
    node = PointCloudNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'timer') and node.timer:
            node.timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()