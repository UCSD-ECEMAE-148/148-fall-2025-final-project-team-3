import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import time
from best_detection_msgs.msg import BestDetection

import sqlite3
from datetime import datetime
import math

NODE_NAME = "hazard_pid_follower"
ACTUATOR_TOPIC_NAME = "/cmd_vel"
BEST_DET_TOPIC = "/best_detection"
GPS_TOPIC = "/gps"
FORWARD_DISTANCE_TOPIC = "/forward_distance"

MIN_DIST_METERS = 5.0

class HazardFollower(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Publishers
        self.twist_pub = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist = Twist()

        # Subscribers
        self.create_subscription(BestDetection, BEST_DET_TOPIC, self.det_callback, 10)
        self.create_subscription(String, GPS_TOPIC, self.gps_callback, 10)
        self.create_subscription(Float32,FORWARD_DISTANCE_TOPIC, self.distance_callback,10)

        self.last_gps = None
        self.target_visible = False
        self.object_low_in_frame = False
        self.current_distance=100
        # Parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("Kp_steering", -0.5),
                ("Ki_steering", 0.0),
                ("Kd_steering", 0.0),
                ("error_threshold", 0.15),
                ("zero_throttle", 0.0),
                ("max_throttle", 0.2),
                ("min_throttle", 0.1),
                ("max_right_steering", 1.0),
                ("max_left_steering", -1.0),
                ("image_width", 640.0),
                ("image_height", 640.0),
            ],
        )

        self.Kp = self.get_parameter("Kp_steering").value
        self.Ki = self.get_parameter("Ki_steering").value
        self.Kd = self.get_parameter("Kd_steering").value
        self.error_threshold = self.get_parameter("error_threshold").value
        self.zero_throttle = self.get_parameter("zero_throttle").value
        self.max_throttle = self.get_parameter("max_throttle").value
        self.min_throttle = self.get_parameter("min_throttle").value
        self.max_right_steering = self.get_parameter("max_right_steering").value
        self.max_left_steering = self.get_parameter("max_left_steering").value

        self.image_width = float(self.get_parameter("image_width").value)
        self.image_height = float(self.get_parameter("image_height").value)

        # PID internals
        self.Ts = 1 / 20.0
        self.ek = 0
        self.ek_1 = 0
        self.integral_error = 0
        self.integral_max = 1e-8


        self.db_path = "/home/projects/ros2_ws/hazard_logs.db"
        self.conn = sqlite3.connect(self.db_path)
        self.cursor = self.conn.cursor()
        self.cursor.execute("""
        CREATE TABLE IF NOT EXISTS hazard_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            label TEXT,
            latitude REAL,
            longitude REAL,
            timestamp TEXT
        )
        """)
        self.conn.commit()
        self.current_label=""
        self.get_logger().info("Hazard PID follower node started.")

    # ------------------------------------------------------------------

    def gps_callback(self, msg):
        self.last_gps = msg.data
    # ------------------------------------------------------------------
    def distance_callback(self,msg):
        print(msg.data)
        self.current_distance=msg.data
    # ------------------------------------------------------------------

    def det_callback(self, msg: BestDetection):
        #        Expected msg.data format:
       # "Oxidizing_substances 0.55 0.25 0.30 0.60 0.80"
        
        
        x1=msg.x1
        x2=msg.x2
        y2=msg.y2
        y1=msg.y1
        self.current_label=msg.class_name

        # Compute center (normalized 0-1)
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        self.get_logger().info("cx:"+str(cx)+"cy:"+str(cy))
        self.target_visible = True

        # Compute error from image center (normalized)
        error = (0.5 - cx)

        # Run PID steering
        self.run_pid(error)

        # Check if object is near bottom of frame
        if self.current_distance<0.9:
            self.object_low_in_frame = True
            self.stop_and_report()

    def run_pid(self, error):
        print(error)

        self.ek = error

        # Throttle gain scheduling
        inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (
            1 - self.error_threshold
        )
        throttle_raw = ((self.min_throttle - self.max_throttle) / (1 - self.error_threshold)) * abs(
            self.ek
        ) + inf_throttle
        throttle = 0.12

        # Steering PID
        proportional = self.Kp * self.ek
        derivative = self.Kd * (self.ek - self.ek_1) / self.Ts
        self.integral_error += self.Ki * self.ek * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)

        steering_raw = proportional + derivative + self.integral_error
        steering = self.clamp(steering_raw, self.max_right_steering, self.max_left_steering)

        # Publish output
        self.twist.angular.z = steering
        self.twist.linear.x = throttle
        self.twist_pub.publish(self.twist)
        self.get_logger().info("Steering Val: "+str(steering))
        self.ek_1 = self.ek

    # ------------------------------------------------------------------

    def stop_and_report(self):
        """Stop robot and print GPS once object is no within distance threshold."""
       # if not self.current_distance<0.5:
       #     return

        # Stop robot
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.twist_pub.publish(self.twist)

        if self.last_gps and self.last_gps != '':
            gps_string = self.last_gps
            gps_split=gps_string.split(',')
            if (len(gps_split)<5):
                return
            try:
                lat=float(gps_split[1])/100
                lon=float(gps_split[3])/100
            
            
                if (gps_split[2]=='S'):
                    lat=-1*lat
                if (gps_split[4]=='W'):
                    lon=-1*lon
                    self.get_logger().info("--------------------------------------------------")
                    self.get_logger().info(f"Target reached. GPS ≈ {lat:.6f}, {lon:.6f}")
                    self.log_hazard_event(
                        label=self.current_label,    # you already extracted label from best_detection
                        lat=lat,
                        lon=lon
                    )
                else:
                    self.get_logger().warn("Target reached but GPS unknown.")
            except Exception as e:
                self.get_logger().warn("GPS ERROR")
        self.object_low_in_frame = False
       # self.twist.linear.x=0.0
       # self.twist.angular.z=0.0
       # self.twist_pub.publish(self.twist)
        rclpy.shutdown()

    # ------------------------------------------------------------------

    def clamp(self, value, upper, lower=None):
        if lower is None:
            lower = -upper
        return max(min(value, upper), lower)
    
    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Earth radius in meters
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    def is_new_hazard(self, label, lat, lon):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute(
            "SELECT latitude, longitude FROM hazard_events WHERE label = ?", 
            (label,)
        )
        rows = cursor.fetchall()
        conn.close()

        # If no hazards of this label exist, it is new
        if len(rows) == 0:
            return True

        for (existing_lat, existing_lon) in rows:
            d = self.haversine(lat, lon, existing_lat, existing_lon)
            if d < MIN_DIST_METERS:
                # Too close — treat as duplicate
                return False

        # Far enough away from all others — it's new
        return True

    def log_hazard_event(self, label, lat, lon):
        if self.is_new_hazard(label, lat, lon):
            ts = datetime.utcnow().isoformat()

            self.cursor.execute("""
                INSERT INTO hazard_events (label, latitude, longitude, timestamp)
                VALUES (?, ?, ?, ?)
            """, (label, lat, lon, ts))

            self.conn.commit()

            self.get_logger().info(
                f"[PID] Logged hazard → label={label}, lat={lat}, lon={lon}, time={ts}"
            )
        else:
            self.get_logger().info(
                f"[PID] Duplicate hazard → label={label}, lat={lat}, lon={lon}"    )

# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = HazardFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping hazard follower...")
        node.twist.linear.x = 0.0
        node.twist.angular.z = 0.0
        node.twist_pub.publish(node.twist)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
