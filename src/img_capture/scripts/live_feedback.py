#!/usr/bin/env python3

import os
import math
import json
import numpy as np
import threading
import tkinter as tk
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String

import matplotlib
matplotlib.use('TkAgg')  # Use the TkAgg backend
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from scipy.ndimage import gaussian_filter

# -------------------------
# Filter function
# -------------------------
def filter_image(img, first_threshold, filter_boost, filter_sigma, second_threshold):
    """
    img is in [0..100] float range.
    1) Zero out values below first_threshold
    2) Add filter_boost to remaining (non-zero) pixels
    3) Apply gaussian blur with sigma=filter_sigma
    4) Zero out values below second_threshold
    5) Binarize anything >0 to 100
    Return the filtered array.
    """
    filtered_img = img.copy()
    filtered_img[filtered_img < first_threshold] = 0
    mask = (filtered_img > 0)
    filtered_img[mask] += filter_boost
    filtered_img = gaussian_filter(filtered_img, sigma=filter_sigma)
    filtered_img[filtered_img < second_threshold] = 0
    # Binarize for display
    visual_img = filtered_img.copy()
    visual_img[visual_img > 0] = 100.0
    return visual_img

# -------------------------
# Node that publishes "live" feedback
# -------------------------
class LiveFeedbackNode(Node):
    """
    Subscribes to camera images (mono16 -> scaled to [0..100]) and odometry.
    Maintains latest camera frame and latest odometry.
    Has a timer at 5 Hz that updates a GUI with:
      1) Normalized image (image / image.max)
      2) Raw scaled image [0..100]
      3) Filtered image [0..100]
    Also shows the latest odometry in a label.
    Has param entries and a Save Params button.
    """

    def __init__(self, freq_hz=5.0):
        super().__init__("live_feedback_node")
        self.get_logger().info("Live Feedback Node started.")

        # Latest camera frame (None if not received yet)
        self.latest_image = None  # float in [0..100]
        # Latest odometry data (parsed from JSON)
        self.latest_odom = None

        # Default filter params
        self.param_file = "detection_params.npy"
        self.first_threshold = 10.0
        self.filter_boost    = 10.0
        self.filter_sigma    = 2.0
        self.second_thresh   = 20.0
        self.load_params()

        # Subscribers
        self.create_subscription(
            Image,
            "camera/thermal_image",
            self.camera_callback,
            10
        )
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.create_subscription(
            String,
            "/combined_odometry",
            self.odom_callback,
            qos_profile
        )

        # Timer at freq_hz to invoke the GUI update callback
        period_s = 1.0 / freq_hz
        self.gui_timer = self.create_timer(period_s, self.gui_update_callback)

    def load_params(self):
        """Load filter parameters from a .npy file if present."""
        if os.path.exists(self.param_file):
            arr = np.load(self.param_file, allow_pickle=True)
            if len(arr) == 4:
                self.first_threshold  = arr[0]
                self.filter_boost     = arr[1]
                self.filter_sigma     = arr[2]
                self.second_thresh    = arr[3]
                self.get_logger().info(f"Loaded filter params: {arr}")
            else:
                self.get_logger().warn("Param file shape mismatch. Using defaults.")
        else:
            # Save defaults
            arr = np.array([self.first_threshold, self.filter_boost,
                            self.filter_sigma, self.second_thresh], dtype=float)
            np.save(self.param_file, arr)
            self.get_logger().info(f"Created default params: {arr}")

    def save_params(self):
        """Save current filter parameters to .npy file."""
        arr = np.array([self.first_threshold, self.filter_boost,
                        self.filter_sigma, self.second_thresh], dtype=float)
        np.save(self.param_file, arr)
        self.get_logger().info(f"Params saved to {self.param_file}: {arr}")

    def camera_callback(self, msg):
        """Receive the camera image, convert to float [0..100]."""
        try:
            raw_16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            scaled = (raw_16.astype(float) / 65535.0) * 100.0
            self.latest_image = scaled
            self.get_logger().debug(f"Received image with shape {scaled.shape} and max {scaled.max():.2f}")
        except ValueError as e:
            self.get_logger().error(f"Failed to decode camera image: {e}")
            return

    def odom_callback(self, msg):
        """Store the latest odometry JSON as a dictionary."""
        try:
            data = json.loads(msg.data)
            self.latest_odom = data  # store as dictionary
            self.get_logger().debug(f"Received odometry data: {data}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse odometry JSON: {e}")

    def gui_update_callback(self):
        """
        Timer callback at 5 Hz to refresh the GUI.
        Updates the plots and odometry data.
        """
        if node_gui is not None:
            node_gui.refresh_gui()

# -------------------------
# The main function w/ GUI
# -------------------------
class LiveFeedbackGUI:
    def __init__(self, node):
        self.node = node

        # Create the Tkinter UI
        self.root = tk.Tk()
        self.root.title("Live Feedback (5 Hz)")

        # 3 subplots in one row
        self.fig = Figure(figsize=(15, 5), dpi=100)
        self.ax1, self.ax2, self.ax3 = self.fig.subplots(1, 3)

        # Create a canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Create dummy images
        dummy_img = np.zeros((10,10), dtype=float)
        self.im_norm    = self.ax1.imshow(dummy_img, cmap='gray', vmin=0, vmax=1)
        self.ax1.set_title("Normalized")

        self.im_raw     = self.ax2.imshow(dummy_img, cmap='gray', vmin=0, vmax=100)
        self.ax2.set_title("Raw [0..100]")

        self.im_filt    = self.ax3.imshow(dummy_img, cmap='gray', vmin=0, vmax=100)
        self.ax3.set_title("Filtered")

        # A small label to show odometry
        self.odom_label = tk.Label(self.root, text="No odom yet", font=("Arial", 10))
        self.odom_label.pack(side=tk.TOP, pady=5)

        # Param area
        self.param_frame = tk.Frame(self.root)
        self.param_frame.pack(side=tk.TOP, pady=5)

        # First Threshold
        tk.Label(self.param_frame, text="First Threshold").grid(row=0, column=0, padx=5)
        self.ent_first_threshold = tk.Entry(self.param_frame, width=6)
        self.ent_first_threshold.insert(0, str(node.first_threshold))
        self.ent_first_threshold.grid(row=0, column=1)

        # Filter Boost
        tk.Label(self.param_frame, text="Filter Boost").grid(row=0, column=2, padx=5)
        self.ent_filter_boost = tk.Entry(self.param_frame, width=6)
        self.ent_filter_boost.insert(0, str(node.filter_boost))
        self.ent_filter_boost.grid(row=0, column=3)

        # Filter Sigma
        tk.Label(self.param_frame, text="Filter Sigma").grid(row=1, column=0, padx=5)
        self.ent_filter_sigma = tk.Entry(self.param_frame, width=6)
        self.ent_filter_sigma.insert(0, str(node.filter_sigma))
        self.ent_filter_sigma.grid(row=1, column=1)

        # Second Threshold
        tk.Label(self.param_frame, text="Second Threshold").grid(row=1, column=2, padx=5)
        self.ent_second_threshold = tk.Entry(self.param_frame, width=6)
        self.ent_second_threshold.insert(0, str(node.second_thresh))
        self.ent_second_threshold.grid(row=1, column=3)

        # Save Params Button
        self.btn_save_params = tk.Button(self.param_frame, text="Save Params", command=self.save_params)
        self.btn_save_params.grid(row=2, column=0, columnspan=4, pady=5)

    def save_params(self):
        """Save filter parameters from the GUI entries to the node."""
        try:
            p1 = float(self.ent_first_threshold.get())
            p2 = float(self.ent_filter_boost.get())
            p3 = float(self.ent_filter_sigma.get())
            p4 = float(self.ent_second_threshold.get())
            self.node.first_threshold = p1
            self.node.filter_boost    = p2
            self.node.filter_sigma    = p3
            self.node.second_thresh   = p4
            self.node.save_params()
            print(f"Saved params: {p1}, {p2}, {p3}, {p4}")
        except ValueError:
            print("Invalid param input, cannot save.")

    def refresh_gui(self):
        """Update the plots and odometry label."""
        print("reached")
        if self.node.latest_image is not None:
            img = self.node.latest_image
            max_val = img.max() if img.size > 0 else 0.0

            # Debug: Print max value
            print(f"[DEBUG] Image max: {max_val:.2f}")

            if max_val < 1e-9:
                norm_img = np.zeros_like(img)
            else:
                norm_img = img / max_val

            raw_img = img

            # Apply filter
            filt_img = filter_image(
                img,
                self.node.first_threshold,
                self.node.filter_boost,
                self.node.filter_sigma,
                self.node.second_thresh
            )

            # Update Normalized Image
            self.im_norm.set_data(norm_img)
            self.ax1.set_title(f"Normalized (max={max_val:.1f})")
            self.im_norm.set_clim(0, 1)

            # Update Raw Image
            self.im_raw.set_data(raw_img)
            self.ax2.set_title(f"Raw [0..100] (max={raw_img.max():.1f})")
            self.im_raw.set_clim(0, 100)

            # Update Filtered Image
            self.im_filt.set_data(filt_img)
            self.ax3.set_title(f"Filtered (max={filt_img.max():.1f}, Spots={np.count_nonzero(filt_img > 0)})")
            self.im_filt.set_clim(0, 100)

            # Refresh the canvas
            self.canvas.draw()

        # Update odometry label
        if self.node.latest_odom is not None:
            odom_text = (
                f"Latest Odom:\n"
                f"Timestamp: {self.node.latest_odom.get('timestamp', 'N/A')}\n"
                f"Latitude: {self.node.latest_odom.get('latitude', 'N/A')}\n"
                f"Longitude: {self.node.latest_odom.get('longitude', 'N/A')}\n"
                f"Altitude: {self.node.latest_odom.get('altitude', 'N/A')}\n"
                f"Pitch: {self.node.latest_odom.get('pitch', 'N/A'):.2f}\n"
                f"Roll: {self.node.latest_odom.get('roll', 'N/A'):.2f}\n"
                f"Yaw: {self.node.latest_odom.get('yaw', 'N/A'):.2f}"
            )
            self.odom_label.config(text=odom_text)

    def run(self):
        """Run the Tkinter main loop."""
        self.root.mainloop()

# -------------------------
# Main function
# -------------------------
def main():
    rclpy.init()
    node = LiveFeedbackNode(freq_hz=5.0)
    gui = LiveFeedbackGUI(node)

    # Start the GUI in the main thread
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Live Feedback Node...")
        rclpy.shutdown()

if __name__ == "__main__":
    main()
