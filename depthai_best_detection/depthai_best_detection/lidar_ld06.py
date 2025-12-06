import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math
import time


class LD06Node(Node):
    def __init__(self):
        super().__init__('lidar_ld06')

        # Serial configuration for LD06 (standard)
        self.port = "/dev/ttyUSB2"
        self.baud = 230400

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
            self.get_logger().info(f"Connected to LD06 LiDAR on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open LD06 port: {e}")
            raise SystemExit

        # Publisher
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # Timer to read data
        self.timer = self.create_timer(0.001, self.read_data)

        # Buffer for packet assembly
        self.buffer = bytearray()


    def read_data(self):
        """Read raw lidar packets and publish as LaserScan."""
        data = self.ser.read(1000)
        if data:
            self.buffer.extend(data)

        # Parse packets when we have enough data
        while len(self.buffer) > 47:
            # Packet header is always 0x54 0x2C
            if self.buffer[0] != 0x54 or self.buffer[1] != 0x2C:
                self.buffer.pop(0)
                continue

            packet = self.buffer[:47]
            self.buffer = self.buffer[47:]

            self.process_packet(packet)


    def process_packet(self, packet):
        """Decode a single LD06 packet and publish LaserScan."""

        # 12 measurement points in each packet
        count = 12  
        angles = []
        distances = []

        # angle is in little endian, scaled by 100
        start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0
        end_angle = struct.unpack('<H', packet[42:44])[0] / 100.0

        # Normalize crossing 360 boundary
        if end_angle < start_angle:
            end_angle += 360.0

        angle_step = (end_angle - start_angle) / (count - 1)

        for i in range(count):
            index = 6 + i * 3
            distance = struct.unpack('<H', packet[index:index+2])[0]

            angle = start_angle + angle_step * i

            distances.append(distance / 1000.0)  # meters
            angles.append(math.radians(angle))

        # Build LaserScan message
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser"

        scan.angle_min = min(angles)
        scan.angle_max = max(angles)
        scan.angle_increment = angles[1] - angles[0]
        scan.range_min = 0.05
        scan.range_max = 12.0
        scan.ranges = distances

        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LD06Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
