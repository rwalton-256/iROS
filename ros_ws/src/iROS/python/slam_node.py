#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 # the message
from sensor_msgs_py import point_cloud2 # the helper functions
import numpy as np
from scipy.spatial import KDTree

# Contains all of the methods for a basic slam implementation
# inits with:
#   Node (ROS2 node)
#   previous_points
#   current_pose -> set to 2D ID matrix
class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        self.previous_points = None
        self.current_pose = np.eye(4)
        self.create_subscription(PointCloud2, '/camera/depth/color/points', self.callback, 10)
        
        self.get_logger().info('SLAM node ready')
    
    def callback(self, msg):
        try:
            # Get points
            points = np.array(list(point_cloud2.read_points(msg, field_names=("x","y","z"), skip_nans=True)), dtype=np.float32)
            if len(points) < 100:
                self.get_logger().warn('Too few points, skipping')
                return
            
            # Remove points too far (likely noise)
            point_distances = np.linalg.norm(points, axis=1)
            points = points[point_distances < 5.0]  # Keep points within 5m
            if len(points) < 100:
                return
            
            # Downsample
            points = points[::10]  # Every 10th point
            
            if self.previous_points is not None and len(self.previous_points) > 50:
                # ICP: Find nearest neighbors
                target_tree = KDTree(points)
                correspondence_distances, correspondence_indices = target_tree.query(self.previous_points)
                
                # Filter bad correspondences
                valid_mask = correspondence_distances < 0.5  # Max 50cm correspondence distance
                if np.sum(valid_mask) < 50:
                    self.get_logger().warn('Too few correspondences')
                    self.previous_points = points
                    return
                
                source_points = self.previous_points[valid_mask]
                target_points = points[correspondence_indices[valid_mask]]
                
                # Compute transformation
                source_centroid = np.mean(source_points, axis=0)
                target_centroid = np.mean(target_points, axis=0)
                cross_covariance = (source_points - source_centroid).T @ (target_points - target_centroid)
                U, _, Vt = np.linalg.svd(cross_covariance)
                rotation = Vt.T @ U.T
                if np.linalg.det(rotation) < 0: 
                    Vt[-1] *= -1
                    rotation = Vt.T @ U.T
                translation = target_centroid - rotation @ source_centroid
                
                # Check if transformation is reasonable
                translation_distance = np.linalg.norm(translation)
                if translation_distance > 0.5:  # Max 50cm movement per frame
                    self.get_logger().warn(f'Large movement: {translation_distance:.2f}m, rejecting')
                    self.previous_points = points
                    return
                
                # Update pose
                transform = np.eye(4)
                transform[:3,:3] = rotation
                transform[:3,3] = translation
                self.current_pose = self.current_pose @ transform
                
                position = self.current_pose[:3,3]
                alignment_error = np.mean(correspondence_distances[valid_mask])
                
                self.get_logger().info(f'Pos: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}] | Error: {alignment_error:.3f}m')
            else:
                self.get_logger().info('Initialized')
            
            self.previous_points = points
            
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