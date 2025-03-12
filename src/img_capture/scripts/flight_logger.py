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
'Flight Logger' node that:
- Subscribes to camera/thermal_image + /combined_odometry
- Queues last N=25 odometry messages
- Has a timer at 3 Hz that checks for the 'latest image', 
  finds best-match odometry, and stores them in lists.
- Saves data every 10 seconda to npy and csv log files
- On shutdown , saves last image and odom data

Made Feb 23 to ensure data logs even when UAV shutdowns unexpectedly
"""

class FlightLoggerNode(Node):
    def __init__(self, odom_queue_size=25, freq_hz=3.0):
        super().__init__("flight_logger_node")
        self.get_logger().info("FlightLoggerNode started.")

        # Odom queue
        self.odom_history = deque(maxlen=odom_queue_size)

        # For saving matched data
        self.saved_images = []   # list of 2D NumPy arrays
        self.saved_odom   = []   # list of dictionaries

        # log directory
        os.makedirs("flight_logs", exist_ok=True)

        # Setup image and odom log files
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.img_filename = f"flight_logs/high_therm_images_{timestamp}.npy"
        self.odom_filename = f"flight_logs/high_flight_odom_{timestamp}.npy"
 
        # Latest camera msg (None if not arrived yet)
        self.latest_img_msg = None

        # Subscribe to camera
        self.img_sub = self.create_subscription(
            Image,
            "camera/thermal_image",
            self.img_callback,
            10
        )

        # Subscribe to odom
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

        # Timer at freq_hz => gather data
        self.timer_period = 1.0 / freq_hz
        self.match_timer = self.create_timer(self.timer_period, self.timer_callback)
        # Timer for 10 sec => log data
        self.log_timer = self.create_timer(10.0, self.save_periodically)

    def img_callback(self, msg):
        """Just store the latest image msg."""
        self.latest_img_msg = msg

    def odom_callback(self, msg):
        """Parse JSON, store in queue."""
        try:
            data = json.loads(msg.data)
            self.odom_history.append(data)  # store raw dict
        except Exception as e:
            self.get_logger().error(f"Failed to parse odom: {e}")

    def timer_callback(self):
        """At 3 Hz, if we have a new image, pair it with the best odometry, store it."""
        if self.latest_img_msg is None:
            return  # no camera yet

        # Convert the image to scaled [0..100]
        msg = self.latest_img_msg
        try:
            raw_16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        except ValueError as e:
            self.get_logger().error(f"Cannot reshape image: {e}")
            return

        scaled_img = (raw_16.astype(float) / 65535.0) * 100.0

        # Find best odom
        img_time_us = (msg.header.stamp.sec * 1_000_000) + (msg.header.stamp.nanosec // 1000)
        best_odom = self.find_closest_odom(img_time_us)
        if best_odom is None:
            # no odom => skip
            self.get_logger().debug("No odom match found for this image. Skipping.")
            return

        # store them
        self.saved_images.append(scaled_img)
        self.saved_odom.append(best_odom)

        # Optionally log something
        self.get_logger().debug(f"Stored 1 image & matching odom. Now have {len(self.saved_images)} samples.")

        # Mark the image as consumed if you want to skip re-using it 
        # (otherwise we'll store it again next cycle).
        self.latest_img_msg = None

    def save_periodically(self):
        """Save images and odometry data every second"""
        if self.saved_images:
            arr = np.array(self.saved_images, dtype=np.float32)
            np.save(self.img_filename, arr)  # Overwrites but maintains continuous saving
            self.get_logger().info(f"Saved {len(self.saved_images)} images to {self.img_filename}")

        if self.saved_odom:
            df = pd.DataFrame(self.saved_odom)
            np.save(self.odom_filename, df.values)
            self.get_logger().info(f"Appended {len(self.saved_odom)} odometry entries to {self.odom_filename}")

    def find_closest_odom(self, img_time_us):
        """Find odom in self.odom_history with closest 'timestamp' field to img_time_us."""
        if not self.odom_history:
            return None

        best = None
        best_diff = float('inf')
        for odom in self.odom_history:
            if "timestamp" not in odom:
                continue
            diff = abs(odom["timestamp"] - img_time_us)
            if diff < best_diff:
                best_diff = diff
                best = odom
        return best

    def save_and_exit(self):
        """On shutdown, save last images & odom to disk."""
        # Save images
        self.save_periodically()
        self.get_logger().info("Final save completed before shutdown")

def main(args=None):
    rclpy.init(args=args)
    node = FlightLoggerNode(odom_queue_size=25, freq_hz=3.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_and_exit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()