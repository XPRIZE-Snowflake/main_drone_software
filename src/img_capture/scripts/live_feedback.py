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
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from scipy.ndimage import gaussian_filter


# ----------------------------------------------------
# Image filtering function (unchanged)
# ----------------------------------------------------
def filter_image(img, first_threshold, filter_boost, filter_sigma, second_threshold):
    """
    img is in [0..100] float range.
    Steps:
     1) Zero out values below first_threshold
     2) Add filter_boost to remaining (non-zero) pixels
     3) Apply gaussian blur with sigma=filter_sigma
     4) Zero out values below second_threshold
     5) Binarize anything >0 to 100
    Return the filtered array.
    """
    filtered_img = img.copy()

    # Step 1
    filtered_img[filtered_img < first_threshold] = 0.0

    # Step 2
    mask = (filtered_img > 0.0)
    filtered_img[mask] += filter_boost

    # Step 3
    filtered_img = gaussian_filter(filtered_img, sigma=filter_sigma)

    # Step 4
    filtered_img[filtered_img < second_threshold] = 0.0

    # Step 5: binarize
    visual_img = filtered_img.copy()
    visual_img[visual_img > 0.0] = 100.0
    return visual_img


# ----------------------------------------------------
# Live Feedback Node
# ----------------------------------------------------
class LiveFeedbackNode(Node):
    """
    Subscribes to:
      - "camera/thermal_image" (Image messages)
      - "/combined_odometry" (JSON-encoded String messages from your new publisher)

    Displays:
      - Three image views (normalized, raw, filtered)
      - A text label with the entire JSON from the latest odometry
    """

    def __init__(self, freq_hz=5.0):
        super().__init__("live_feedback_node")
        self.get_logger().info("Live Feedback Node started.")

        # Latest camera frame (None if not received yet)
        self.latest_image = None  # float in [0..100]

        # Latest odometry data (stored as a Python dictionary from JSON)
        self.latest_odom = None

        # Latest hotspot location
        self.hotspot_location = None

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
        self.hot_sub = self.create_subscription(
            String, "hotspot_info", 
            self.hot_callback, 10
        )

        #publishers
        self.drop_mech_pub = self.create_publisher(
            String, "/move_command", 10
        )

        # Timer at freq_hz => call a GUI update callback
        period_s = 1.0 / freq_hz
        self.gui_timer = self.create_timer(period_s, self.gui_update_callback)

    def load_params(self):
        """Load filter parameters from a .npy file if present, or create with defaults."""
        if os.path.exists(self.param_file):
            arr = np.load(self.param_file, allow_pickle=True)
            if len(arr) == 4:
                self.first_threshold  = arr[0]
                self.filter_boost     = arr[1]
                self.filter_sigma     = arr[2]
                self.second_thresh    = arr[3]
            else:
                self.get_logger().error("Param file shape mismatch. Using defaults.")
        else:
            # Save defaults if no file
            arr = np.array([self.first_threshold, self.filter_boost,
                            self.filter_sigma, self.second_thresh], dtype=float)
            np.save(self.param_file, arr)

    def save_params(self):
        """Save current filter parameters to .npy file."""
        arr = np.array([self.first_threshold, self.filter_boost,
                        self.filter_sigma, self.second_thresh], dtype=float)
        np.save(self.param_file, arr)
        self.get_logger().info(f"Params saved to {self.param_file}")

    def camera_callback(self, msg: Image):
        """Receive the camera image, convert to float in [0..100]."""
        try:
            raw_16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        except ValueError as e:
            self.get_logger().error(f"Failed to decode camera image: {e}")
            return
        scaled = (raw_16.astype(float) / 65535.0) * 100.0
        self.latest_image = scaled

    def odom_callback(self, msg: String):
        """
        Parse JSON from /combined_odometry.
        Example fields:
          timestamp, latitude_deg, longitude_deg, altitude_ellipsoid_m,
          vel_n_m_s, vel_e_m_s, vel_d_m_s, eph, epv, s_variance_m_s,
          heading_accuracy, qw, qx, qy, qz.
        """
        try:
            data = json.loads(msg.data)
            self.latest_odom = data
        except Exception as e:
            self.get_logger().error(f"Failed to parse odometry JSON: {e}")

    def hot_callback(self, msg):
        try: 
            data = json.loads(msg.data)
            self.hot_location = data
        except Exception as e:
            self.get_logger().error(f"Failed to parse hotspot data in mech drop code: {e}")

    def gui_update_callback(self):
        """
        Timer callback at 5 Hz to refresh the GUI.
        We'll handle the actual drawing in a function in main() that uses Tk + Matplotlib.
        """
        # This node method just runs. The real "drawing" occurs in main().
        pass


# ----------------------------------------------------
# Main function with Tk GUI
# ----------------------------------------------------
def main():
    rclpy.init()
    node = LiveFeedbackNode(freq_hz=5.0)

    # Spin in a background thread so that callbacks keep working
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Create a Tkinter window
    root = tk.Tk()
    root.title("Live Feedback (5 Hz)")

    # Make a 3-subplot figure for (Normalized, Raw, Filtered)
    fig = Figure(figsize=(12,4), dpi=100)
    ax1, ax2, ax3 = fig.subplots(1, 3)

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    # Initialize dummy images
    dummy_img = np.zeros((10,10), dtype=float)

    im_norm = ax1.imshow(dummy_img, cmap='gray', vmin=0, vmax=1, origin="lower")
    ax1.set_title("Normalized")

    im_raw = ax2.imshow(dummy_img, cmap='gray', vmin=0, vmax=100, origin="lower")
    ax2.set_title("Raw [0..100]")

    im_filt = ax3.imshow(dummy_img, cmap='gray', vmin=0, vmax=100, origin="lower")
    ax3.set_title("Filtered")

    # A label to show odometry
    odom_label = tk.Label(root, text="No odom yet", font=("Arial", 10))
    odom_label.pack(side=tk.TOP, pady=5)

    # The label to show hotspot location
    hot_label = tk.Label(root, text="No hotspot found", font=("Ariel", 10))
    hot_label.pack(side=tk.TOP, pady=5)

    # Param controls
    param_frame = tk.Frame(root)
    param_frame.pack(side=tk.TOP, pady=5)

    tk.Label(param_frame, text="First Threshold").grid(row=0, column=0, padx=5)
    ent_first_threshold = tk.Entry(param_frame, width=6)
    ent_first_threshold.insert(0, str(node.first_threshold))
    ent_first_threshold.grid(row=0, column=1)

    tk.Label(param_frame, text="Filter Boost").grid(row=0, column=2, padx=5)
    ent_filter_boost = tk.Entry(param_frame, width=6)
    ent_filter_boost.insert(0, str(node.filter_boost))
    ent_filter_boost.grid(row=0, column=3)

    tk.Label(param_frame, text="Filter Sigma").grid(row=1, column=0, padx=5)
    ent_filter_sigma = tk.Entry(param_frame, width=6)
    ent_filter_sigma.insert(0, str(node.filter_sigma))
    ent_filter_sigma.grid(row=1, column=1)

    tk.Label(param_frame, text="Second Threshold").grid(row=1, column=2, padx=5)
    ent_second_threshold = tk.Entry(param_frame, width=6)
    ent_second_threshold.insert(0, str(node.second_thresh))
    ent_second_threshold.grid(row=1, column=3)

    def save_params():
        try:
            node.first_threshold = float(ent_first_threshold.get())
            node.filter_boost    = float(ent_filter_boost.get())
            node.filter_sigma    = float(ent_filter_sigma.get())
            node.second_thresh   = float(ent_second_threshold.get())
            node.save_params()
        except ValueError:
            print("Invalid param input, cannot save.")

    tk.Button(param_frame, text="Save Params", command=save_params).grid(
        row=2, column=0, columnspan=4, pady=5
    )

    def drop_mech():
        command = String()
        command.data = "open"
        node.drop_mech_pub.publish(command)
        node.get_logger().info(f"Opened drop mechanism")
    
    tk.Button(param_frame, text="Drop", command=drop_mech).grid(
        row=2, column=2, columnspan=4, pady=5
    )

    # GUI refresh function, called ~5 Hz
    def refresh_gui():
        # 1) If we have a new image, update the plots
        if node.latest_image is not None:
            img = node.latest_image
            max_val = img.max() if img.size > 0 else 0.0

            # Normalized
            if max_val > 1e-9:
                norm_img = (img - img.min()) / (max_val - img.min())
            else:
                norm_img = np.zeros_like(img)

            im_norm.set_data(norm_img)
            im_norm.set_clim(0, 1)
            ax1.set_title(f"Normalized (max={max_val:.1f})")

            # Raw
            im_raw.set_data(img)
            im_raw.set_clim(0, 100)

            # Filtered
            filt_img = filter_image(
                img,
                node.first_threshold,
                node.filter_boost,
                node.filter_sigma,
                node.second_thresh
            )
            im_filt.set_data(filt_img)
            im_filt.set_clim(0, 100)

        # 2) Display latest odometry (raw JSON)
        if node.latest_odom is not None:
            txt = f"Latest Odom:\n{json.dumps(node.latest_odom, indent=2)}"
            odom_label.config(text=txt)
        # 3) Display hotspot relative location
        if node.hotspot_location is not None:
            hot_txt = f"Hotspot location: {json.dumps(node.hot_location, indent=2)}\n"
            hot_label.config(text=hot_txt)

        canvas.draw()
        # Schedule next update
        root.after(200, refresh_gui)  # ~5 Hz = 200 ms

    # Start the periodic GUI update
    root.after(200, refresh_gui)

    def on_closing():
        node.get_logger().info("Shutting down live_feedback_node ...")
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()