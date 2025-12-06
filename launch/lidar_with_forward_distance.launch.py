from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LD06 LiDAR driver
        Node(
            package="depthai_best_detection",
            executable="lidar_ld06",
            name="lidar",
            output="screen",
        ),

        # Forward distance processor
        Node(
            package="depthai_best_detection",
            executable="forward_distance_node",
            name="forward_distance",
            output="screen",
            parameters=[{
                "scan_topic": "/scan",
                "angle_window": 5.0
            }]
        ),
    ])
