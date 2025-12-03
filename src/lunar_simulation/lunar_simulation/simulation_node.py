#!/usr/bin/env python3
"""
ROS 2 Node: Lunar Simulation Manager
Publishes simulated sensor data for crater-based localization
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
import numpy as np

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')
        
        # Publishers (as per proposal data pipeline)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/wheel_encoders', 10)
        self.depth_pub = self.create_publisher(PointCloud2, '/gazebo/depth_camera', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/localization/pose_estimate', 10)
        self.crater_pub = self.create_publisher(PoseStamped, '/perception/crater_detections', 10)
        
        # Timer for simulation updates (10Hz as per proposal)
        self.timer = self.create_timer(0.1, self.publish_simulation_data)  # 10Hz
        
        self.get_logger().info('🚀 Lunar Simulation Node Started')
        self.get_logger().info('📊 Publishing topics:')
        self.get_logger().info('   /odometry/wheel_encoders (50Hz)')
        self.get_logger().info('   /gazebo/depth_camera (30Hz)')
        self.get_logger().info('   /localization/pose_estimate (10Hz)')
        self.get_logger().info('   /perception/crater_detections (10Hz)')
        
        # Simulation state
        self.sim_time = 0.0
        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        
    def publish_simulation_data(self):
        """Publish all sensor data as per proposal"""
        self.sim_time += 0.1
        
        # Publish odometry (with simulated 2% drift)
        self.publish_odometry()
        
        # Publish depth data (simulated)
        self.publish_depth_data()
        
        # Publish pose estimate (simulated particle filter output)
        self.publish_pose_estimate()
        
        # Publish crater detections (simulated)
        self.publish_crater_detections()
        
    def publish_odometry(self):
        """Publish wheel odometry data with 2% drift (as per proposal)"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Simulate position with drift
        drift_factor = 1.02  # 2% drift
        self.position[0] += 0.1 * drift_factor  # X position
        self.position[1] += 0.05 * drift_factor  # Y position
        
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]
        
        self.odom_pub.publish(odom_msg)
        
    def publish_depth_data(self):
        """Publish simulated depth point cloud"""
        # Create a simple PointCloud2 message (simplified for demo)
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'depth_camera'
        
        # Generate some random points (simulating crater points)
        points = []
        for _ in range(100):
            x = np.random.uniform(-5, 5)
            y = np.random.uniform(-5, 5)
            z = np.random.uniform(0, 10)
            points.append([x, y, z])
            
        # In real implementation, would create proper PointCloud2
        # For demo, just log that we would publish
        if self.sim_time % 5.0 < 0.1:  # Log every 5 seconds
            self.get_logger().info(f'📷 Depth camera data simulated: {len(points)} points')
            
    def publish_pose_estimate(self):
        """Publish particle filter pose estimate"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Simulated particle filter output (better than odometry)
        pf_x = self.position[0] * 0.98  # 2% correction from particle filter
        pf_y = self.position[1] * 0.98
        
        pose_msg.pose.position.x = pf_x
        pose_msg.pose.position.y = pf_y
        pose_msg.pose.position.z = 0.0
        
        self.pose_pub.publish(pose_msg)
        
    def publish_crater_detections(self):
        """Publish detected crater features"""
        if self.sim_time % 2.0 < 0.1:  # Simulate crater detection every 2 seconds
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            
            # Simulate detecting a crater
            pose_msg.pose.position.x = np.random.uniform(2, 8)
            pose_msg.pose.position.y = np.random.uniform(-4, 4)
            pose_msg.pose.position.z = 0.0
            
            self.crater_pub.publish(pose_msg)
            self.get_logger().info(f'📍 Crater detected at ({pose_msg.pose.position.x:.1f}, {pose_msg.pose.position.y:.1f})')

def main(args=None):
    rclpy.init(args=args)
    node = SimulationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
