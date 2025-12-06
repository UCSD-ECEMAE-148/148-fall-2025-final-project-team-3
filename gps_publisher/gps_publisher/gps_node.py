import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

GPS_SERIAL = '/dev/ttyUSB0'      # your working port
GPS_BAUD = 460800                # FE baud

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.pub = self.create_publisher(String, 'gps', 10)
        self.get_logger().info("GPS Node Started")

        self.ser = serial.Serial(GPS_SERIAL, GPS_BAUD, timeout=1)

        # read at 10Hz
        self.timer = self.create_timer(0.1, self.read_gps)

    def read_gps(self):
        try:
            raw = self.ser.readline()

            # attempt ASCII decode but ignore any bad bytes
            try:
                line = raw.decode('ascii', errors='ignore')
            except:
                return

            if line.startswith('$GNGLL'):
                msg = String()
                msg.data = line.strip()
                self.pub.publish(msg)
                self.get_logger().info(line.strip())

        except Exception as e:
            self.get_logger().error(f"Error reading GPS: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
