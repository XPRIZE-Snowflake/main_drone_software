#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleAttitude, SensorGps
from std_msgs.msg import String
import json

class CombinedOdometryPublisher(Node):
    def __init__(self):
        super().__init__('combined_odometry_publisher')
        self.get_logger().info("Combined Odometry Publisher started.\n")

        # -- Internal storage for raw data
        # We'll store them as Python floats or integers,
        # so we don't risk "float32 is not JSON serializable" errors.
        self.timestamp = 0

        # GPS data
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude_ellipsoid = 0.0
        self.vel_n_m_s = 0.0
        self.vel_e_m_s = 0.0
        self.vel_d_m_s = 0.0
        self.eph = 0.0
        self.epv = 0.0
        self.s_variance_m_s = 0.0
        self.heading_accuracy = 0.0

        # Attitude quaternion
        self.qw = 1.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0

        # Set up QoS profile for PX4 topics (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscriptions
        self.create_subscription(VehicleAttitude,
                                 '/fmu/out/vehicle_attitude',
                                 self.attitude_callback,
                                 qos_profile)

        self.create_subscription(SensorGps,
                                 '/fmu/out/vehicle_gps_position',
                                 self.gps_callback,
                                 qos_profile)

        # Publisher: combined odometry data
        self.publisher = self.create_publisher(String, '/combined_odometry', 10)

        # Timer for periodic publishing (20 Hz)
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

    def attitude_callback(self, msg: VehicleAttitude):
        """
        Callback for VehicleAttitude messages to retrieve raw quaternion (body->NED).
        """
        # Ensure we cast each to Python float
        q = msg.q  # [w, x, y, z]
        self.qw = float(q[0])
        self.qx = float(q[1])
        self.qy = float(q[2])
        self.qz = float(q[3])

    def gps_callback(self, msg: SensorGps):
        """
        Callback for SensorGps messages to retrieve raw data:
        timestamp, lat, lon, alt ellipsoid, velocities, accuracy, etc.
        """
        self.timestamp = int(msg.timestamp)  # microseconds (as int)
        self.latitude = float(msg.latitude_deg)
        self.longitude = float(msg.longitude_deg)

        # altitude above ellipsoid
        self.altitude_ellipsoid = float(msg.altitude_ellipsoid_m)

        # Raw GPS velocities in NED
        self.vel_n_m_s = float(msg.vel_n_m_s)
        self.vel_e_m_s = float(msg.vel_e_m_s)
        self.vel_d_m_s = float(msg.vel_d_m_s)

        # Accuracy metrics
        self.eph = float(msg.eph)
        self.epv = float(msg.epv)
        self.s_variance_m_s = float(msg.s_variance_m_s)
        self.heading_accuracy = float(msg.heading_accuracy)

    def timer_callback(self):
        """
        Periodic publish of combined data as a JSON string.
        """
        data_dict = {
            "timestamp": self.timestamp,
            "latitude_deg": self.latitude,
            "longitude_deg": self.longitude,
            "altitude_ellipsoid_m": self.altitude_ellipsoid,
            "vel_n_m_s": self.vel_n_m_s,
            "vel_e_m_s": self.vel_e_m_s,
            "vel_d_m_s": self.vel_d_m_s,
            "eph": self.eph,
            "epv": self.epv,
            "s_variance_m_s": self.s_variance_m_s,
            "heading_accuracy": self.heading_accuracy,
            "qw": self.qw,
            "qx": self.qx,
            "qy": self.qy,
            "qz": self.qz
        }

        # Convert dict to JSON
        json_msg = String()
        json_msg.data = json.dumps(data_dict)

        # Publish
        self.publisher.publish(json_msg)

        # Log (optional)
        self.get_logger().debug(
            f"GPS timestamp={self.timestamp}, "
            f"lat={self.latitude}, lon={self.longitude}, alt_ellipsoid={self.altitude_ellipsoid}, "
            f"vel_n={self.vel_n_m_s}, vel_e={self.vel_e_m_s}, vel_d={self.vel_d_m_s}, "
            f"eph={self.eph}, epv={self.epv}, s_variance={self.s_variance_m_s}, heading_acc={self.heading_accuracy}, "
            f"quat=({self.qw}, {self.qx}, {self.qy}, {self.qz})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CombinedOdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()