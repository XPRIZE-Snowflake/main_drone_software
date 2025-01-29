#!/usr/bin/env python3

import os
import numpy as np
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import matplotlib
matplotlib.use('TkAgg')  # Use the TkAgg backend for matplotlib
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from scipy.ndimage import gaussian_filter

# -------------------------
# Filtering function
# -------------------------
def filter_image(img, first_threshold, filter_boost, filter_sigma, second_threshold, camera_dims):
    """
    1) Zero out values below first_threshold
    2) Add filter_boost to remaining (non-zero) pixels
    3) Apply gaussian blur with sigma=filter_sigma
    4) Zero out values below second_threshold
    5) Produce visual_img (white=255 where >0, black=0 otherwise)
    6) Find "hot spots" by repeated max search & clearing around it
    """
    # Make a float copy
    filtered_img = img.astype(float)

    # 1) Apply first threshold
    filtered_img[filtered_img < first_threshold] = 0

    # 2) Add boost to non-zero
    mask_nonzero = (filtered_img > 0)
    filtered_img[mask_nonzero] += filter_boost

    # 3) Gaussian blur
    filtered_img = gaussian_filter(filtered_img, sigma=filter_sigma)

    # 4) Second threshold
    filtered_img[filtered_img < second_threshold] = 0

    # 5) Make a visual copy
    visual_img = filtered_img.copy()
    visual_img[visual_img > 0] = 255

    # 6) Locate hot spots
    hot_spots = []
    weights = []
    black_out_edge = int(np.ceil(2 * filter_sigma))

    # Make a separate array for “peak finding”
    search_img = filtered_img.copy()

    while search_img.max() > 0:
        # find the maximum
        hot_spot = np.unravel_index(np.argmax(search_img), search_img.shape)
        # sum of the region near that hotspot
        lower_x = max(0, hot_spot[0] - black_out_edge)
        upper_x = min(camera_dims[0], hot_spot[0] + black_out_edge + 1)
        lower_y = max(0, hot_spot[1] - black_out_edge)
        upper_y = min(camera_dims[1], hot_spot[1] + black_out_edge + 1)

        weights.append(np.sum(search_img[lower_x:upper_x, lower_y:upper_y]))
        # zero out that area
        search_img[lower_x:upper_x, lower_y:upper_y] = 0
        hot_spots.append(hot_spot)

    return visual_img.astype(np.uint8), np.array(hot_spots).astype(int), np.array(weights)

# -------------------------
# ROS Node (single‐image)
# -------------------------
class ThermalImageSubscriber(Node):
    """
    ROS 2 node that subscribes to 'camera/thermal_image' (mono8).
    We only care about the *next* image after 'update_requested' is True.
    """
    def __init__(self):
        super().__init__('thermal_subscriber_np')

        self.update_requested = False
        self.current_image = None

        # Let the GUI block on this event after request_next_image() is called
        self.image_ready_event = threading.Event()

        # Subscription
        self.subscription = self.create_subscription(
            Image,
            'camera/thermal_image',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        """Only accept the next frame if 'update_requested' is True; discard otherwise."""
        if not self.update_requested:
            return

        # Convert from mono8
        try:
            img_array = np.frombuffer(msg.data, dtype=np.uint8)
            img_array = img_array.reshape(msg.height, msg.width)
        except ValueError as e:
            self.get_logger().error(f"Failed to convert image data to NumPy array: {e}")
            return

        self.current_image = img_array

        # The next image was captured
        self.update_requested = False
        self.image_ready_event.set()

    def request_next_image(self):
        """Called by GUI to ask for the next (live) frame."""
        self.image_ready_event.clear()
        self.update_requested = True

    def wait_for_new_image(self, timeout=3.0):
        """Blocks up to 'timeout' for a new image."""
        received = self.image_ready_event.wait(timeout)
        if not received:
            return None
        return self.current_image

# -------------------------
# Main GUI Application
# -------------------------
def main():
    # 1) Load or create parameter file
    param_file = "detection_params.npy"
    if os.path.exists(param_file):
        # load parameters
        params = np.load(param_file, allow_pickle=True)
        # Expect [first_threshold, filter_boost, filter_sigma, second_threshold]
        if len(params) != 4:
            print(f"Parameter file {param_file} has unexpected shape; re-init with defaults.")
            params = np.array([100, 100, 2, 35], dtype=float)
            np.save(param_file, params)
    else:
        # create with defaults
        params = np.array([100, 100, 2, 35], dtype=float)
        np.save(param_file, params)

    detection_first_threshold    = params[0]
    detection_filter_boost       = params[1]
    detection_filter_sigma       = params[2]
    detection_second_threshold   = params[3]

    # 2) Initialize ROS and start background spinning
    rclpy.init()
    node = ThermalImageSubscriber()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # 3) Build the Tkinter UI with side-by-side images
    root = tk.Tk()
    root.title("Thermal Viewer: Unfiltered + Filtered")

    fig = Figure(figsize=(8,4), dpi=100)
    # Create 1 row, 2 columns subplot
    (ax1, ax2) = fig.subplots(1,2)

    # Dummy data
    dummy_img = np.zeros((10,10), dtype=np.uint8)
    im_unfiltered = ax1.imshow(dummy_img, cmap='gray', vmin=0, vmax=255)
    ax1.set_title("Unfiltered")

    im_filtered = ax2.imshow(dummy_img, cmap='gray', vmin=0, vmax=255)
    ax2.set_title("Filtered")

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack()

    # 4) Parameter entries
    # We'll create 4 label+entry pairs
    param_frame = tk.Frame(root)
    param_frame.pack(side=tk.TOP, pady=5)

    tk.Label(param_frame, text="First Threshold").grid(row=0, column=0, padx=5)
    ent_first_threshold = tk.Entry(param_frame, width=6)
    ent_first_threshold.insert(0, str(detection_first_threshold))
    ent_first_threshold.grid(row=0, column=1)

    tk.Label(param_frame, text="Filter Boost").grid(row=0, column=2, padx=5)
    ent_filter_boost = tk.Entry(param_frame, width=6)
    ent_filter_boost.insert(0, str(detection_filter_boost))
    ent_filter_boost.grid(row=0, column=3)

    tk.Label(param_frame, text="Filter Sigma").grid(row=1, column=0, padx=5)
    ent_filter_sigma = tk.Entry(param_frame, width=6)
    ent_filter_sigma.insert(0, str(detection_filter_sigma))
    ent_filter_sigma.grid(row=1, column=1)

    tk.Label(param_frame, text="Second Threshold").grid(row=1, column=2, padx=5)
    ent_second_threshold = tk.Entry(param_frame, width=6)
    ent_second_threshold.insert(0, str(detection_second_threshold))
    ent_second_threshold.grid(row=1, column=3)

    def save_params():
        """Save the 4 param values to file."""
        try:
            p1 = float(ent_first_threshold.get())
            p2 = float(ent_filter_boost.get())
            p3 = float(ent_filter_sigma.get())
            p4 = float(ent_second_threshold.get())
            np.save(param_file, np.array([p1, p2, p3, p4]))
            print(f"Saved params to {param_file}")
        except ValueError:
            print("Invalid input in param fields. Could not save.")

    tk.Button(param_frame, text="Save Params", command=save_params).grid(row=2, column=0, columnspan=4, pady=5)

    # 5) Update button logic
    def update_plot():
        """
        1) Request next live frame
        2) Wait for it
        3) Display unfiltered on left
        4) Filter and display on right
        """
        # parse param fields
        try:
            first_thresh  = float(ent_first_threshold.get())
            boost         = float(ent_filter_boost.get())
            sigma         = float(ent_filter_sigma.get())
            second_thresh = float(ent_second_threshold.get())
        except ValueError:
            print("Cannot parse param fields. Aborting update.")
            return

        # 1) request next image
        node.request_next_image()

        # 2) wait up to 3 seconds
        new_image = node.wait_for_new_image(timeout=3.0)
        if new_image is None:
            print("No new image within 3 seconds.")
            return

        # 3) show unfiltered
        im_unfiltered.set_data(new_image)
        ax1.set_title(f"Unfiltered: {new_image.shape}")
        im_unfiltered.set_clim(0, 255)

        # 4) filter
        camera_dims = new_image.shape
        filtered_img, hot_spots, weights = filter_image(
            new_image,
            first_thresh,
            boost,
            sigma,
            second_thresh,
            camera_dims
        )
        im_filtered.set_data(filtered_img)
        ax2.set_title(f"Filtered: {filtered_img.shape}\nHot spots: {len(hot_spots)}")
        im_filtered.set_clim(0, 255)

        canvas.draw()

    btn_update = tk.Button(root, text="Update", command=update_plot)
    btn_update.pack(pady=5)

    # 6) Cleanup on close
    def on_closing():
        node.get_logger().info("Closing GUI, shutting down ROS...")
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    root.mainloop()


if __name__ == '__main__':
    main()
