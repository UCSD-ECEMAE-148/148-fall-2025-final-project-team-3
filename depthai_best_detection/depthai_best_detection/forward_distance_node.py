import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np


class ForwardDistanceNode(Node):
    def __init__(self):
        super().__init__('forward_distance')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(Float32, '/forward_distance', 10)

        # Smoothing factor for exponential moving average
        self.alpha = 0.3
        self.filtered_distance = None

    def scan_callback(self, msg: LaserScan):

        # Convert scan to numpy array
        ranges = np.array(msg.ranges)

        # Remove invalid values
        ranges = ranges[np.isfinite(ranges)]
        ranges = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]

        if ranges.size == 0:
            return

        # Calculate the angle for each beam
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Select beams within ±5 degrees of 0 (forward)
        fwd_mask = np.abs(angles) < np.radians(5.0)
        forward_ranges = ranges[fwd_mask]

        if forward_ranges.size == 0:
            return

        # Remove noisy spikes using median
        forward_distance = float(np.median(forward_ranges))

        # Apply smoothing
        if self.filtered_distance is None:
            self.filtered_distance = forward_distance
        else:
            self.filtered_distance = (
                self.alpha * forward_distance
                + (1 - self.alpha) * self.filtered_distance
            )

        # Publish
        msg_out = Float32()
        msg_out.data = self.filtered_distance
        self.pub.publish(msg_out)

        self.get_logger().info(
            f"Forward distance (filtered): {self.filtered_distance:.3f} m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ForwardDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
