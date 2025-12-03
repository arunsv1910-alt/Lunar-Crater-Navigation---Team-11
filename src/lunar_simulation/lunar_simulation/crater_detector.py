#!/usr/bin/env python3
"""
ROS 2 Node: Crater Detector
Processes depth data to detect crater landmarks
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
import numpy as np

class CraterDetector(Node):
    def __init__(self):
        super().__init__('crater_detector')
        
        # Subscriber to depth data
        self.depth_sub = self.create_subscription(
            PointCloud2,
            '/gazebo/depth_camera',
            self.depth_callback,
            10)
        
        # Publisher for detected craters
        self.craters_pub = self.create_publisher(
            PoseArray,
            '/perception/detected_craters',
            10)
        
        self.get_logger().info('🔍 Crater Detector Node Started')
        self.get_logger().info('   Subscribing to: /gazebo/depth_camera')
        self.get_logger().info('   Publishing to: /perception/detected_craters')
        
    def depth_callback(self, msg):
        """Process depth data to detect craters"""
        # Simulated crater detection algorithm
        # In real implementation, would process PointCloud2
        
        # Simulate detecting 3-5 craters (as per proposal)
        num_craters = np.random.randint(3, 6)
        
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'base_link'
        
        for i in range(num_craters):
            from geometry_msgs.msg import Pose
            pose = Pose()
            
            # Simulated crater positions (within sensor range)
            pose.position.x = np.random.uniform(5, 15)
            pose.position.y = np.random.uniform(-8, 8)
            pose.position.z = 0.0
            
            # Simulated crater radius (5-10m as per proposal)
            # Stored in orientation quaternion (x field for radius)
            radius = np.random.uniform(2.5, 5.0)
            pose.orientation.x = radius  # Using x to store radius
            
            pose_array.poses.append(pose)
        
        self.craters_pub.publish(pose_array)
        self.get_logger().info(f'📍 Detected {num_craters} craters (simulated)')

def main(args=None):
    rclpy.init(args=args)
    node = CraterDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
