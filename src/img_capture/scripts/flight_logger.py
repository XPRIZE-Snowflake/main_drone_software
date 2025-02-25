#!/usr/bin/env python3

import os
import math
import json
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String

from collections import deque
from datetime import datetime

"""
Flight logger node that:
- Subscribes to "camera/thermal_image" (Image messages) 
- Subscribes to "/combined_odometry" (JSON-encoded String messages)
- Queues the last N=25 odometry messages
- Uses a timer at 3 Hz to:
    1. Check if we have a 'latest' camera image
    2. Find the best-match odometry (closest timestamp)
    3. Store them in lists
- On shutdown, saves images and odometry to disk as .npy
"""

class LoggerNode(Node):
    def __init__(self, odom_queue_size=25, freq_hz=3.0):
        super().__init__("logger_node")
        self.get_logger().info("LoggerNode started.")

        # Keep a history of recent odometry for timestamp matching
        self.odom_history = deque(maxlen=odom_queue_size)

        # Storage for matched image-odom pairs
        self.saved_images = []  # list of 2D NumPy arrays (thermal images)
        self.saved_odom   = []  # list of dicts (JSON from combined_odometry)

        # Keep the latest camera message (or None if none yet)
        self.latest_img_msg = None

        # Subscription to camera images
        self.img_sub = self.create_subscription(
            Image,
            "camera/thermal_image",
            self.img_callback,
            10
        )

        # Subscription to the combined odometry JSON
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.odom_sub = self.create_subscription(
            String,
            "/combined_odometry",
            self.odom_callback,
            qos_profile
        )

        # Timer at freq_hz => try to match image + odometry
        self.timer_period = 1.0 / freq_hz
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)

    def img_callback(self, msg: Image):
        """
        Store the latest image message in memory.
        """
        self.latest_img_msg = msg

    def odom_callback(self, msg: String):
        """
        Parse the JSON from /combined_odometry and store in a queue.
        The message is a JSON string with fields:
          { 
            "timestamp": ..., "latitude_deg": ..., "longitude_deg": ...,
            "altitude_ellipsoid_m": ..., "vel_n_m_s": ..., ...
            "qw": ..., "qx": ..., "qy": ..., "qz": ...
          }
        """
        try:
            data = json.loads(msg.data)
            self.odom_history.append(data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse odom JSON: {e}")

    def timer_callback(self):
        """
        Every 1/freq_hz seconds, if we have a new image,
        match it with the closest odometry by timestamp, and store the pair.
        """
        if self.latest_img_msg is None:
            return  # No new image available

        msg = self.latest_img_msg
        # Convert 16-bit image data to float array
        try:
            raw_16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        except ValueError as e:
            self.get_logger().error(f"Cannot reshape image: {e}")
            return

        # Scale image to [0..100] just as an example
        scaled_img = (raw_16.astype(float) / 65535.0) * 100.0

        # Convert image timestamp to microseconds
        img_time_us = (msg.header.stamp.sec * 1_000_000) + (msg.header.stamp.nanosec // 1000)

        # Find best odom match
        best_odom = self.find_closest_odom(img_time_us)
        if best_odom is None:
            self.get_logger().info("No odom match found for this image. Skipping.")
            return

        # Store the matched pair
        self.saved_images.append(scaled_img)
        self.saved_odom.append(best_odom)

        # Log
        self.get_logger().info(
            f"Stored 1 image & matching odom. Total pairs: {len(self.saved_images)}."
        )

        # Mark this image as consumed so we don't reuse it next cycle
        self.latest_img_msg = None

    def find_closest_odom(self, img_time_us: int):
        """
        Search self.odom_history for the entry with 'timestamp' closest to img_time_us.
        Return that dictionary, or None if no match.
        """
        if not self.odom_history:
            return None

        best = None
        best_diff = float('inf')
        for odom_dict in self.odom_history:
            if "timestamp" not in odom_dict:
                continue
            diff = abs(odom_dict["timestamp"] - img_time_us)
            if diff < best_diff:
                best_diff = diff
                best = odom_dict
        return best

    def save_and_exit(self):
        """
        On shutdown, dump the collected pairs to disk as .npy files.
        """
        # Save images (N x H x W array)
        arr = np.array(self.saved_images, dtype=np.float32)
        img_filename = f"flight_logs/flight_images_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npy"
        os.makedirs(os.path.dirname(img_filename), exist_ok=True)
        np.save(img_filename, arr)
        self.get_logger().info(f"Saved {len(self.saved_images)} images to {img_filename}")

        # Save odom: build a DataFrame, then save .npy of the numeric data
        df = pd.DataFrame(self.saved_odom)
        odom_filename = f"flight_logs/flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npy"
        os.makedirs(os.path.dirname(odom_filename), exist_ok=True)
        np.save(odom_filename, df.values)
        self.get_logger().info(f"Saved {len(self.saved_odom)} odometry entries to {odom_filename}")

def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode(odom_queue_size=25, freq_hz=3.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On shutdown, save all matched images & odometry
        node.save_and_exit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

