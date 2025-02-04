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
from scipy.ndimage import gaussian_filter



class LowAltFilterNode(Node):
    def __init__(self, odom_queue_size=25, freq_hz=3.0):
        super().__init__("low_alt_filter_node")
        self.get_logger().info("LowAltFilterNode started.")

        ## data taken from topics ##
        self.odom_history = deque(maxlen=odom_queue_size)
        self.saved_images = []
        self.saved_odom = []
        self.latest_img_msg = None

        self.home_location = None
        self.sim = False
        self.pitch =0
        self.roll = 0
        self.yaw = 0
        self.altitude = 0

        ## Camera Intrinsic Characteristics ##
        self.fov_x = 56
        self.fov_y = 45
        self.alt = 100
        #self.camera_dims = None
        self.first_threshold = 100
        self.filter_boost = 100
        self.filter_sigma = 2
        self.second_threshold = 35

        ## Subscribers ##
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

        ## Publishers ##
        self.hotspot_pub = self.create_publisher(String, 'hotspot_info', 10)


    def img_callback(self, msg):
        self.latest_img_msg = msg
	

    def odom_callback(self, msg):
        try:
            data = json.loads(msg.data)

	    # If this is the first odometry message, save it as the home location
            if not hasattr(self, "home_location"):
                self.home_location = data
                self.sim = all(value == 0 for value in data.values())
                self.get_logger().info(f"Home location set: {self.home_location}, Sim mode: {self.sim}")

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
        filtered_img = gaussian_filter(scaled_img, sigma=self.filter_sigma)

	    ## Find hotspot
        hotspot_y, hotspot_x = np.unravel_index(np.argmax(filtered_img), filtered_img.shape)

        img_time_us = (msg.header.stamp.sec * 1_000_000) + (msg.header.stamp.nanosec // 1000)
        best_odom = self.find_closest_odom(img_time_us)
        if best_odom is None:
            self.get_logger().info("No odom match found for this image. Skipping.")
            return
        
        center_x, center_y = msg.width // 2, msg.height // 2
        pixel_offset_x, pixel_offset_y = hotspot_x - center_x, hotspot_y - center_y

        # ## take odom data and locate hotspots
        self.altitude = 100 
        #self.altitude = best_odom.get("altitude", 100)
        self.pitch = math.radians(best_odom.get("pitch", 0))
        self.roll = math.radians(best_odom.get("roll", 0))
        self.yaw = math.radians(best_odom.get("yaw", 0))

        # Compare altitude with home location
        if(self.home_location is None):
            self.home_altitude = 100
        else:
            self.home_altitude = self.home_location.get("altitude", 100)  # Default home altitude
        altitude_difference = self.altitude - self.home_altitude

        # Camera properties
        FOV_x = math.radians(56)  # Example: Adjust based on your camera's FOV (horizontal)
        FOV_y = math.radians(45)  # Example: Adjust based on vertical FOV
        img_width, img_height = msg.width, msg.height

        # # Calculate relative hotspot position
        # relative_hotspot = self.calculate_hotspot_location([(hotspot_x, hotspot_y)], (img_width, img_height))

        # # Log results
        # self.get_logger().info(
        #     f"Hotspot relative to camera: {relative_hotspot} | Altitude Difference: {altitude_difference:.2f}m"
        # )

        # Convert pixel offset to angle offsets
        angle_offset_x = (pixel_offset_x / img_width) * FOV_x
        angle_offset_y = (pixel_offset_y / img_height) * FOV_y

        # Compute ground distances using altitude and pitch
        dx = self.altitude * math.tan(angle_offset_x)
        dy = self.altitude * math.tan(angle_offset_y)

        # Apply pitch and roll corrections
        adjusted_x = dx * math.cos(self.pitch) + dy * math.sin(self.roll)
        adjusted_y = dy * math.cos(self.roll) - dx * math.sin(self.pitch)

        # Rotate by yaw to align with world coordinates
        world_x = adjusted_x * math.cos(self.yaw) - adjusted_y * math.sin(self.yaw)
        world_y = adjusted_x * math.sin(self.yaw) + adjusted_y * math.cos(self.yaw)
        
        self.get_logger().info(f"Hotspot in world coords: X={world_x:.2f}, Y={world_y:.2f}, Alt={self.altitude:.2f}")

        # Calculate relative hotspot position
        relative_hotspot = self.calculate_hotspot_location([(hotspot_x, hotspot_y)], (img_width, img_height))
        # Log results
        self.get_logger().info(
            f"Hotspot relative to camera: {relative_hotspot} | Altitude Difference: {altitude_difference:.2f}m"
        )
        
        self.saved_images.append(scaled_img)
        self.saved_odom.append(best_odom)
        self.latest_img_msg = None

    def calculate_hotspot_distance(self, x, y, image_width, image_height, fov_x, fov_y, altitude):
        # Convert FOV to radians
        fov_x_rad = np.radians(fov_x)
        fov_y_rad = np.radians(fov_y)

        center_x, center_y = image_width / 2, image_height / 2
        pixel_offset_x = x - center_x
        pixel_offset_y = y - center_y
    
        # Calculate angular offsets
        theta_x = (pixel_offset_x / image_width)  * fov_x_rad
        theta_y = (pixel_offset_y / image_height) * fov_y_rad
    
        # Compute forward distance
        forward_distance = self.home_altitude / np.cos(self.pitch)  # Adjusted altitude-based distance

        # Calculate ground distances
        dx = forward_distance * np.tan(theta_x)
        dy = forward_distance * np.tan(theta_y)

        # Adjust using pitch and roll
        adjusted_x = dx * np.cos(self.pitch) + dy * np.sin(self.roll)
        adjusted_y = dy * np.cos(self.roll) - dx * np.sin(self.pitch)

        # Compute final 3D Euclidean distance from the camera to the hotspot
        distance = np.sqrt(adjusted_x**2 + adjusted_y**2 + self.home_altitude**2)
    
        return distance
    
    def calculate_hotspot_location(self, hot_spots, camera_dims):
        if len(hot_spots) > 0:
            # Find the hottest hotspot
            hottest_spot = hot_spots[0]  # The first hotspot is the hottest due to the filtering logic
            x, y = hottest_spot

            # Center of the image
            center_x, center_y = camera_dims[0] // 2, camera_dims[1] // 2

            # Calculate distances from the center of the image in meters
            horizontal_distance = self.calculate_hotspot_distance(
                x, center_y, camera_dims[0], camera_dims[1], self.fov_x, self.fov_y, self.alt
            )
            vertical_distance = self.calculate_hotspot_distance(
                center_x, y, camera_dims[0], camera_dims[1], self.fov_x, self.fov_y, self.alt
            )

            # Determine vertical direction
            if y > center_y:
                vertical_dir = f"{vertical_distance:.2f} meters down"
            elif y < center_y:
                vertical_dir = f"{vertical_distance:.2f} meters up"
            else:
                vertical_dir = "centered vertically"

            # Determine horizontal direction
            if x > center_x:
                horizontal_dir = f"{horizontal_distance:.2f} meters right"
            elif x < center_x:
                horizontal_dir = f"{horizontal_distance:.2f} meters left"
            else:
                horizontal_dir = "centered horizontally"

            # Create the message
            message = (
                f"Hottest Hotspot: Coordinates=({x}, {y}), "
                f"Vertical Offset={vertical_dir}, "
                f"Horizontal Offset={horizontal_dir}"
            )

            self.get_logger().info(message)
            # Publish the message to the ROS topic
            self.hotspot_pub.publish(String(data=message))
        else:
            self.get_logger().info("No hotspot detected")


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