#!/usr/bin/env python3

## comparing home location to world coordinates. distance is constantly around 100

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
        # self.altitude = 1400 
        self.altitude = best_odom.get("altitude", 1400)
        self.pitch = math.radians(best_odom.get("pitch", 0))
        self.roll = math.radians(best_odom.get("roll", 0))
        self.yaw = math.radians(best_odom.get("yaw", 0))

        # Compare altitude with home location
        if(self.home_location is None):
            self.home_altitude = 1300
        else:
            self.home_altitude = self.home_location.get("altitude", 1300)  # Default home altitude
        altitude_difference = self.altitude - self.home_altitude

        # Camera properties
        FOV_x = math.radians(56)  # Example: Adjust based on your camera's FOV (horizontal)
        FOV_y = math.radians(42)  # Example: Adjust based on vertical FOV
        img_width, img_height = msg.width, msg.height

        ## Find world coordinate
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
        # self.calculate_hotspot_location([(hotspot_x, hotspot_y)], (img_width, img_height))

        #camera to world 
        x, y = self.pixels_to_world(np.array([[hotspot_x, hotspot_y]]) , np.array([240, 320]), np.array([56, 42]), np.array([self.pitch]), np.array([self.roll]), np.array([0]), np.array([0, 0, altitude_difference]))[0]
        
        self.get_logger().info(
            f"Relative coords: X:{x}, Y: {y} Altitude Difference: {altitude_difference:.2f}m"
        )

        # Log results
        self.get_logger().info(
            f"First Alt: {self.home_altitude}, Curr Alt: {self.altitude} Altitude Difference: {altitude_difference:.2f}m"
        )
        
        self.saved_images.append(scaled_img)
        self.saved_odom.append(best_odom)
        self.latest_img_msg = None

    def pixels_to_world(self, pixel_coords, camera_dims, fov, pitch, roll, yaw, camera_coords):
        # Convert degrees to radians
        fov_rad = fov * np.pi / 180  # fov is now [fov_x, fov_y]
        fovx_rad, fovy_rad = fov_rad

        pitch_rad = pitch
        roll_rad = roll
        yaw_rad = yaw

        # Calculate rotation matrix
        R_roll = np.array([
            [np.cos(roll_rad), 0, np.sin(roll_rad)],
            [0,                 1, 0                ],
            [-np.sin(roll_rad), 0, np.cos(roll_rad)]
        ])
        R_pitch = np.array([
            [1, 0,                 0                ],
            [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
            [0, np.sin(pitch_rad),  np.cos(pitch_rad)]
        ])
        R_yaw = np.array([
            [np.cos(yaw_rad), np.sin(yaw_rad), 0],
            [-np.sin(yaw_rad),  np.cos(yaw_rad), 0],
            [0,               0,               1]
        ])
        R = R_yaw @ R_pitch @ R_roll

        # Calculate pixel ratios
        pixel_ratios = pixel_coords / (camera_dims - 1)

        # Calculate angle ratios
        angle_x = (pixel_ratios[:, 0] - 0.5) * fovx_rad
        angle_y = (pixel_ratios[:, 1] - 0.5) * fovy_rad

        # Calculate direction in the camera space
        sin_angle_x = np.sin(angle_x)
        sin_angle_y = np.sin(angle_y)
        cos_angle_x = np.cos(angle_x)
        cos_angle_y = np.cos(angle_y)
        dir_x = sin_angle_x * cos_angle_y
        dir_y = sin_angle_y
        dir_z = -cos_angle_x * cos_angle_y
        direction_camera_space = np.stack((dir_x, dir_y, dir_z), axis=-1)
        direction_camera_space /= np.linalg.norm(direction_camera_space, axis=1, keepdims=True)

        # Calculate the direction in the world space
        direction_world_space = (R @ direction_camera_space.T).T
        direction_world_space /= np.linalg.norm(direction_world_space, axis=1, keepdims=True)

        # Calculate the ground coordinates
        t = -camera_coords[2] / direction_world_space[:, 2]
        ground_coords = camera_coords[:2] + direction_world_space[:, :2] * t[:, np.newaxis]

        # Return the ground coordinates
        return ground_coords

    def calculate_hotspot_position(self, x, y, image_width, image_height, fov_x, fov_y, altitude, pitch, roll, yaw):
        """
        Calculate the real-world x, y, and z distances of a hotspot from the camera, 
        considering pitch, roll, and yaw.
        """
        # Convert angles to radians
        pitch = np.radians(pitch)
        roll = np.radians(roll)
        yaw = np.radians(yaw)
        fov_x_rad = np.radians(fov_x)
        fov_y_rad = np.radians(fov_y)

        # Compute the image center and pixel offsets
        center_x, center_y = image_width / 2, image_height / 2
        pixel_offset_x = x - center_x
        pixel_offset_y = y - center_y

        # Compute angular offsets from the center
        theta_x = (pixel_offset_x / image_width) * fov_x_rad
        theta_y = (pixel_offset_y / image_height) * fov_y_rad

        # Compute forward distance based on pitch
        forward_distance = altitude / np.cos(pitch)  # Adjust altitude-based distance

        # Compute distances in camera frame
        dx_camera = forward_distance * np.tan(theta_x)
        dy_camera = forward_distance * np.tan(theta_y)
        dz_camera = altitude  # Direct altitude difference

        # Apply pitch and roll transformations
        adjusted_x = dx_camera * np.cos(pitch) + dy_camera * np.sin(roll)
        adjusted_y = dy_camera * np.cos(roll) - dx_camera * np.sin(pitch)

        # Transform to world frame using yaw
        world_x = adjusted_x * np.cos(yaw) - adjusted_y * np.sin(yaw)
        world_y = adjusted_x * np.sin(yaw) + adjusted_y * np.cos(yaw)
        world_z = dz_camera  # Z remains altitude

        return world_x, world_y, world_z

    
    def calculate_hotspot_location(self, hot_spots, camera_dims):
        if len(hot_spots) > 0:
            # Find the hottest hotspot
            hottest_spot = hot_spots[0]  # The first hotspot is the hottest due to the filtering logic
            x, y = hottest_spot

            # Center of the image
            center_x, center_y = camera_dims[0] // 2, camera_dims[1] // 2
            
            vertical_distance, horizontal_distance, alt = self.calculate_hotspot_position(
                x, y, camera_dims[0], camera_dims[1], self.fov_x, self.fov_y, self.alt, self.pitch, self.roll, self.yaw
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
                f"Relative Hottest Hotspot: Pixel Coordinates=({x}, {y}), "
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