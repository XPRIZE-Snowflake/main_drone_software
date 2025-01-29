#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleAttitude, SensorGps
from std_msgs.msg import String
import math
import json
import pandas as pd
from datetime import datetime

class CustomPoseMsg:
    def __init__(self, timestamp, latitude, longitude, altitude, pitch, roll, yaw):
        self.timestamp = timestamp
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

class CombinedOdometryPublisher(Node):
    def __init__(self, save_data=False, max_saved_data_points=1000000):
        super().__init__('combined_odometry_publisher')

        # Initialize variables
        self.roll = self.pitch = self.yaw = 0.0
        self.latitude = self.longitude = self.altitude = 0.0
        self.timestamp = 0

        # Set up QoS profile for subscriptions (BEST_EFFORT for PX4 topics)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Create subscriptions
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position', self.gps_position_callback, qos_profile)

        # Create publisher for combined odometry data
        self.publisher = self.create_publisher(String, '/combined_odometry', 10)

        # Timer for periodic publishing at 20 Hz (50 ms interval)
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

        # Create dictionary to save data if enabled
        self.saved_data = {
            "timestamp": [],
            "latitude": [],
            "longitude": [],
            "altitude": [],
            "pitch": [],
            "roll": [],
            "yaw": []
        }
        self.save_data = save_data
        self.max_saved_data_points = max_saved_data_points

        # Save start time
        self.start_time = datetime.now().strftime("%Y:%m:%d-%H:%M:%S")

    def timer_callback(self):
        """Timer callback to publish combined odometry data."""
        self.publish_combined_odometry()

    def attitude_callback(self, msg):
        """Callback for VehicleAttitude messages to extract roll, pitch, yaw."""
        w, x, y, z = msg.q  # Quaternion components from VehicleAttitude message
        self.roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        self.pitch = math.asin(2 * (w * y - z * x))
        self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def gps_position_callback(self, msg):
        """Callback for SensorGps messages to extract GPS data."""
        self.latitude = msg.latitude_deg
        self.longitude = msg.longitude_deg
        self.altitude = msg.altitude_msl_m
        self.timestamp = msg.timestamp

    def publish_combined_odometry(self):
        """Publish combined odometry data as a JSON string."""
        custom_pose = CustomPoseMsg(
            timestamp=self.timestamp,
            latitude=self.latitude,
            longitude=self.longitude,
            altitude=self.altitude,
            pitch=self.pitch,
            roll=self.roll,
            yaw=self.yaw
        )

        # Save data if enabled
        if self.save_data:
            self.saved_data["timestamp"].append(self.timestamp)
            self.saved_data["latitude"].append(self.latitude)
            self.saved_data["longitude"].append(self.longitude)
            self.saved_data["altitude"].append(self.altitude)
            self.saved_data["pitch"].append(self.pitch)
            self.saved_data["roll"].append(self.roll)
            self.saved_data["yaw"].append(self.yaw)

            # Trim saved data if it exceeds the maximum allowed points
            if len(self.saved_data["timestamp"]) > self.max_saved_data_points:
                for key in self.saved_data:
                    self.saved_data[key].pop(0)

        # Convert CustomPoseMsg to JSON string and publish it
        json_str = json.dumps(custom_pose.__dict__)
        msg = String()
        msg.data = json_str

        # Publish the message on the /combined_odometry topic
        self.publisher.publish(msg)

        # Log
        self.get_logger().info(f"timestamp: {self.timestamp}, latitude: {self.latitude}, longitude: {self.longitude}, altitude: {self.altitude}, pitch: {self.pitch}, roll: {self.roll}, yaw: {self.yaw}")

    def save_data_to_csv(self, file_name="odometry_data"):
        """Save saved data to a CSV file."""
        end_time = datetime.now().strftime("%Y:%m:%d-%H:%M:%S")
        pd.DataFrame(self.saved_data).to_csv(f"{file_name}_{self.start_time}_to_{end_time}.csv", index=False)

def main(args=None):
    rclpy.init(args=args)
    node = CombinedOdometryPublisher(save_data=True, max_saved_data_points=1000000)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data_to_csv(file_name="odometry_data")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
