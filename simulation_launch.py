#!/usr/bin/env python3
"""
ROS 2 Launch file for Lunar Rover Simulation
Launches complete simulation stack as per proposal
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
from launch.substitutions import FindExecutable

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="🚀 LAUNCHING LUNAR ROVER SIMULATION - TEAM 11"),
        LogInfo(msg="=============================================="),
        
        # 1. Start Gazebo with 15-crater world
        ExecuteProcess(
            cmd=[
                'gz', 'sim', '-v', '4',
                'worlds/lunar_crater_world_15.sdf'
            ],
            output='screen',
            name='gazebo_sim',
            shell=True
        ),
        
        # 2. Simulation Node (sensor data)
        Node(
            package='lunar_simulation',
            executable='simulation_node',
            name='simulation_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/odometry/wheel_encoders', '/odom'),
                ('/gazebo/depth_camera', '/depth/points'),
                ('/localization/pose_estimate', '/pf/pose'),
                ('/perception/crater_detections', '/craters/detected')
            ]
        ),
        
        # 3. Crater Detector Node
        Node(
            package='lunar_simulation',
            executable='crater_detector',
            name='crater_detector',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 4. Localization Node (Particle Filter)
        Node(
            package='lunar_simulation',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'num_particles': 200},
                {'resampling_threshold': 0.5}
            ]
        ),
        
        LogInfo(msg="=============================================="),
        LogInfo(msg="✅ SIMULATION STACK LAUNCHED"),
        LogInfo(msg="   • Gazebo with 15 craters"),
        LogInfo(msg="   • ROS 2 nodes running"),
        LogInfo(msg="   • Data pipeline active"),
        LogInfo(msg="   • Ready for localization"),
        LogInfo(msg="=============================================="),
    ])
