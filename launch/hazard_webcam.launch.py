#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # -----------------------------
    # CAMERA + CPU YOLO DETECTOR
    # -----------------------------
    camera_node = Node(
        package='depthai_best_detection',
        executable='webcam_cpu_detector',
        name='webcam_cpu_detector',
        output='screen',
        parameters=[{
            'model_path': '/home/projects/ros2_ws/models/hazard.pt',
            'show_visualization': True
        }]
    )

    # -----------------------------
    # PID FOLLOWER NODE
    # -----------------------------
    pid_node = Node(
        package='depthai_best_detection',
        executable='hazard_pid_follower',
        name='hazard_pid_follower',
        output='screen',
        parameters=[{
            'image_width': 640.0,
            'image_height': 640.0
        }]
    )

    # -----------------------------
    # GPS NODE
    # (You must replace pkg + exec)
    # -----------------------------
    gps_node = Node(
        package='gps_publisher',       # <--- REPLACE
        executable='gps_node',        # <--- REPLACE
        name='gps_node',
        output='screen'
    )

    # -----------------------------
    # VESC TWIST NODE
    # (From ucsd_robocar_actuator2_pkg)
    # -----------------------------
    vesc_node = Node(
        package='ucsd_robocar_actuator2_pkg',
        executable='vesc_twist_node',
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel')   # leave unchanged unless needed
        ],
        parameters=[{
            # --- USER-REQUESTED OVERRIDES ---
            'max_rpm': 20000,
            'steering_polarity': 1,
            'throttle_polarity': 1,

            # Steering mapping
            'max_right_steering': 1.0,
            'straight_steering': 0.0,
            'max_left_steering': -1.0,

            # Throttle
            'zero_throttle': 0.0,
            'max_throttle': 1.0,
            'min_throttle': 1.0
        }]
    )

    return LaunchDescription([
        camera_node,
        pid_node,
        gps_node,
        vesc_node
    ])
