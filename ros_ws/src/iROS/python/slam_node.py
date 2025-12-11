#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import sensor_msgs.msg
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2 # the message

import re
import numpy as np
import threading

from scipy.spatial import KDTree

# Contains all of the methods for a basic slam implementation
# inits with:
#   Node (ROS2 node)
#   previous_points
#   current_pose -> set to 2D ID matrix
class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        self.check_topics()

        # Parameter declarations, can be modified in the launch file
        self.declare_parameter('iphone_name', 'iPhone')
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('downsample_factor', 10)
        self.declare_parameter('focal_length_x', 256.0)
        self.declare_parameter('focal_length_y', 192.0)
        self.declare_parameter('principal_point_x', 128.0)
        self.declare_parameter('principal_point_y', 96.0)

        self.max_depth = self.get_parameter('max_depth').value
        self.downsample_factor = self.get_parameter('downsample_factor').value
        self.focal_length_x = self.get_parameter('focal_length_x').value
        self.focal_length_y = self.get_parameter('focal_length_y').value
        self.center_x = self.get_parameter('principal_point_x').value
        self.center_y = self.get_parameter('principal_point_y').value

        self.previous_points = None
        self.current_pose = np.eye(4) # Identity matrix, AKA default rotation, translation
        # self.create_subscription(PointCloud2, '/camera/depth/color/points', self.callback, 10)

        self.prev_state = {}
        self.subs = []
        
        self.get_logger().info('SLAM node ready')

        self.check_topics()
    
    def check_topics(self):        
        for node_name in self.get_node_names():
            for topic_name, _ in self.get_publisher_names_and_types_by_node(node_name, ''):
                if re.match(r"^/iphone/.*/eo$",topic_name):
                    if not topic_name in self.pubs.keys():
                        self.get_logger().info(f"Creating subscription for {topic_name}!")
                        self.pubs[topic_name] = self.create_publisher(
                            sensor_msgs.msg.Image,
                            topic_name.replace("/eo","/eo_slam"),
                            10
                        )
                        self.subs.append(
                            self.create_subscription(
                                sensor_msgs.msg.Image,
                                topic_name,
                                lambda msg, topic=topic_name: self.process_frame(topic, msg),
                                10
                            )
                        )
        self.timer = threading.Timer(1,self.check_topics).start()
    
    # Initializes the state of the device
    def initialize_device(self, topic_name):
        """Initialize state and subscription for a new device"""
        # Create state for this device
        self.device_state[topic_name] = {
            'previous_points': None,
            'current_pose': np.eye(4)
        }
        
        # Create subscription
        self.subscriptions.append(
            self.create_subscription(
                Image,
                topic_name,
                lambda msg, topic=topic_name: self.depth_callback(topic, msg),
                10
            )
        )

    # Convert image data (depth map) to a point cloud
    # Includes: Filtering, downsampling
    def depth_map_to_points(self, depth_map):
        height, width = depth_map.shape
        point_cloud_data = []
        
        # Downsample at source - only process every Nth pixel
        for pixel_row in range(0, height, self.downsample_factor):
            for pixel_col in range(0, width, self.downsample_factor):
                depth_value = depth_map[pixel_row, pixel_col]
                
                # Filter: valid depth within range
                if depth_value > 0 and depth_value < self.max_depth:
                    # Unproject to 3D using pinhole camera model
                    point_x = (pixel_col - self.center_x) * depth_value / self.focal_length_x
                    point_y = (pixel_row - self.center_y) * depth_value / self.focal_length_y
                    point_z = depth_value
                    point_cloud_data.append([point_x, point_y, point_z])
        
        return np.array(point_cloud_data, dtype=np.float32) if point_cloud_data else np.array([])

    # Process depth maps from multiple devices
    # Devices are referenced by topic
    # Includes multiple pass filtering:
    #   ICP: Iterative Closest Point
    #           - Aligns sensor scans
    #   CDT: Correspondance Distance Threshold
    #           - Corrects for point mismatching
    def depth_callback(self, topic_name, msg):
        try:
            # Retrieve state from given device
            state = self.device_state[topic_name]
            device_name = topic_name.split('/')[2]  # Extract device name from topic
            
            # Convert ROS Image to numpy array
            if msg.encoding != "32FC1":
                self.get_logger().error(f'[{device_name}] Unexpected encoding: {msg.encoding}')
                return
            
            depth_map = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            
            # Convert to point cloud (with built-in filtering and downsampling)
            points = self.depth_map_to_points(depth_map)
            
            if len(points) < 100:
                self.get_logger().warn(f'[{device_name}] Too few points after filtering')
                return
            
            if state['previous_points'] is not None and len(state['previous_points']) > 50:
                # ICP: Find nearest neighbors
                target_tree = KDTree(points)
                correspondence_distances, correspondence_indices = target_tree.query(state['previous_points'])
                
                # Filter bad correspondences
                #   Bad correspondances are classified as points in the same spot
                #   that have exceeded a difference value
                #   This should probably be a parameter... but it's set to 50cm
                #   If there aren't enough correspondences for ICP, use the old points
                valid_mask = correspondence_distances < 0.5
                if np.sum(valid_mask) < 50:
                    self.get_logger().warn(f'[{device_name}] Too few correspondences')
                    state['previous_points'] = points
                    return
                
                source_points = state['previous_points'][valid_mask]
                target_points = points[correspondence_indices[valid_mask]]
                
                # Extract centroids of both point clouds
                # https://arxiv.org/pdf/2206.06435
                # This is some pretty in-the-weeds math
                # A relatively quick rundown:
                #   Align using the centroids of the previous state and the current state
                #   The cross-variance matrix is used to calculate an optimal rotation
                #   Said optimal rotation is found by using singular value decomposition 
                #   on the cross-variance matrix
                source_centroid = np.mean(source_points, axis=0) # The previous state's point cloud
                target_centroid = np.mean(target_points, axis=0) # The state to transform the "source" to
                cross_covariance = (source_points - source_centroid).T @ (target_points - target_centroid)
                U, _, Vt = np.linalg.svd(cross_covariance)
                rotation = Vt.T @ U.T
                if np.linalg.det(rotation) < 0:
                    Vt[-1] *= -1
                    rotation = Vt.T @ U.T
                translation = target_centroid - rotation @ source_centroid
                
                # Check for and discard movements over large distances
                #   Note: half a meter is a CRAZY distance to move during most computational windows
                translation_distance = np.linalg.norm(translation)
                if translation_distance > 0.5:
                    self.get_logger().warn(f'[{device_name}] Large movement: {translation_distance:.2f}m, rejecting')
                    state['previous_points'] = points
                    return
                
                # Update pose
                transform = np.eye(4) # no rotation, at origin
                transform[:3,:3] = rotation # extracted rotation from the cross-variance and SVD
                transform[:3,3] = translation # extracted translation from ^
                state['current_pose'] = state['current_pose'] @ transform
                
                position = state['current_pose'][:3,3]
                alignment_error = np.mean(correspondence_distances[valid_mask])
                
                self.get_logger().info(
                    f'[{device_name}] Pos: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}] | '
                    f'Error: {alignment_error:.3f}m | Points: {len(points)}'
                )
            else:
                self.get_logger().info(f'[{device_name}] Initialized')
            
            state['previous_points'] = points
            
        except Exception as e:
            self.get_logger().error(f'SLAM error: {e}')

def main():
    rclpy.init()
    node = SlamNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()