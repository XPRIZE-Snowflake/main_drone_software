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

class LowAltFilterNode(Node):
    def __init__(self, odom_queue_size=25, freq_hz=3.0):
        super().__init__("low_alt_filter_node")
        self.get_logger().info("LowAltFilterNode started.")

        self.odom_history = deque(maxlen=odom_queue_size)
        self.saved_images = []
        self.saved_odom = []
        self.latest_img_msg = None

        self.img_sub = self.create_subscription(
            Image, "camera/thermal_image", self.img_callback, 10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=10
        )
        self.odom_sub = self.create_subscription(
            String, "/combined_odometry", self.odom_callback, qos_profile
        )

        self.timer_period = 1.0 / freq_hz
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)

    def img_callback(self, msg):
        self.latest_img_msg = msg

    def odom_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.odom_history.append(data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse odom: {e}")

    def timer_callback(self):
        if self.latest_img_msg is None:
            return

        msg = self.latest_img_msg
        try:
            raw_16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        except ValueError as e:
            self.get_logger().error(f"Cannot reshape image: {e}")
            return

        scaled_img = (raw_16.astype(float) / 65535.0) * 100.0

        hotspot_y, hotspot_x = np.unravel_index(np.argmax(scaled_img), scaled_img.shape)
        center_x, center_y = msg.width // 2, msg.height // 2
        pixel_offset_x, pixel_offset_y = hotspot_x - center_x, hotspot_y - center_y

        img_time_us = (msg.header.stamp.sec * 1_000_000) + (msg.header.stamp.nanosec // 1000)
        best_odom = self.find_closest_odom(img_time_us)
        if best_odom is None:
            self.get_logger().info("No odom match found for this image. Skipping.")
            return

        altitude = 100 #best_odom.get("altitude", 100)
        pitch = math.radians(best_odom.get("pitch", 0))
        roll = math.radians(best_odom.get("roll", 0))
        yaw = math.radians(best_odom.get("yaw", 0))
        
        meters_per_pixel = altitude / 100.0  # Example: Adjust based on actual camera specs
        offset_x_meters = pixel_offset_x * meters_per_pixel
        offset_y_meters = pixel_offset_y * meters_per_pixel

        adjusted_x = offset_x_meters * math.cos(yaw) - offset_y_meters * math.sin(yaw)
        adjusted_y = offset_x_meters * math.sin(yaw) + offset_y_meters * math.cos(yaw)

        self.get_logger().info(
            f"Hotspot offset (m): X={adjusted_x:.2f}, Y={adjusted_y:.2f}, Alt={altitude:.2f}"
        )

        self.saved_images.append(scaled_img)
        self.saved_odom.append(best_odom)
        self.latest_img_msg = None

    def find_closest_odom(self, img_time_us):
        if not self.odom_history:
            return None

        best, best_diff = None, float('inf')
        for odom in self.odom_history:
            if "timestamp" not in odom:
                continue
            diff = abs(odom["timestamp"] - img_time_us)
            if diff < best_diff:
                best_diff = diff
                best = odom
        return best

    def save_and_exit(self):
        arr = np.array(self.saved_images, dtype=np.float32)
        img_filename = f"low_alt_images_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npy"
        np.save(img_filename, arr)
        self.get_logger().info(f"Saved {len(self.saved_images)} images to {img_filename}")

        df = pd.DataFrame(self.saved_odom)
        odom_filename = f"low_alt_odom_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        df.to_csv(odom_filename, index=False)
        self.get_logger().info(f"Saved {len(self.saved_odom)} odometry entries to {odom_filename}")


def main(args=None):
    rclpy.init(args=args)
    node = LowAltFilterNode(odom_queue_size=25, freq_hz=3.0)

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