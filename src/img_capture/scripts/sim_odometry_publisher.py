#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import json
import threading
import time

class CustomPoseMsg:
    def __init__(self, timestamp, latitude, longitude, altitude, pitch, roll, yaw):
        self.timestamp = timestamp
        self.latitude  = latitude
        self.longitude = longitude
        self.altitude  = altitude
        self.pitch     = pitch
        self.roll      = roll
        self.yaw       = yaw

class SimCombinedOdometryPublisher(Node):
    def __init__(self):
        super().__init__('sim_combined_odometry_publisher')
        self.publisher = self.create_publisher(String, '/combined_odometry', 10)

        # Load simulation odometry data.
        # Expected shape: (num_rows, 7) where columns are:
        # [timestamp, latitude, longitude, altitude, pitch, roll, yaw]
        try:
            self.odom_data = np.load('sim_data/clean_odometry.npy')
        except Exception as e:
            self.get_logger().error(f"Failed to load simulation odometry data: {e}")
            return

        if self.odom_data.ndim != 2 or self.odom_data.shape[1] != 7:
            self.get_logger().error("Odometry data must be a 2D array with 7 columns (timestamp, lat, lon, alt, pitch, roll, yaw).")
            return

        # Determine the publishing frequency from timestamps (which are in microseconds).
        timestamps = self.odom_data[:, 0].astype(np.int64)
        if timestamps.shape[0] > 1:
            dt_us = np.diff(timestamps)
            avg_dt_us = np.mean(dt_us)
            freq = 1e6 / avg_dt_us
            self.get_logger().info(f"Estimated publishing frequency from odometry timestamps: {freq:.2f} Hz")
        else:
            self.get_logger().warn("Only one odometry data point found; frequency estimation is not possible.")

        # Start a thread that will publish the odometry messages using the timestamp differences.
        self.publish_thread = threading.Thread(target=self.publish_odometry)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def publish_odometry(self):
        num_rows = self.odom_data.shape[0]
        for i in range(num_rows):
            row = self.odom_data[i]
            # Unpack the row; ensure that the timestamp is an integer.
            timestamp = int(row[0])
            latitude  = float(row[1])
            longitude = float(row[2])
            altitude  = float(row[3])
            pitch     = float(row[4])
            roll      = float(row[5])
            yaw       = float(row[6])

            # Create the custom pose message and convert to JSON.
            custom_pose = CustomPoseMsg(timestamp, latitude, longitude, altitude, pitch, roll, yaw)
            json_str = json.dumps(custom_pose.__dict__)
            msg = String()
            msg.data = json_str

            # Publish on the /combined_odometry topic.
            self.publisher.publish(msg)
            self.get_logger().info(f"Published odometry: {json_str}")

            # If not at the last row, sleep for the appropriate interval.
            if i < num_rows - 1:
                # Compute the delay in seconds based on the difference in microsecond timestamps.
                next_timestamp = int(self.odom_data[i + 1, 0])
                dt_sec = (next_timestamp - timestamp) / 1e6
                time.sleep(dt_sec)

def main(args=None):
    rclpy.init(args=args)
    node = SimCombinedOdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down simulation node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
