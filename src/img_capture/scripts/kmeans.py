#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from std_msgs.msg import String

class DynamicKMeans(Node):
  def __init__(self, max_radius, min_size):
    """
    Initialize the dynamic k-means object.
    :param max_radius: The maximum radius to consider a point close to a centroid.
    :param min_size: Minimum number of elements required for a centroid to be returned.
    """
    super().__init__('dynamic_kmeans')
    self.max_radius = max_radius
    self.min_size = min_size
    self.centroids = np.empty((0, 2))  # Start with no centroids
    self.elements = np.empty((0, 2))  # Start with no elements
    self.labels = []  # Store labels for the most recent batch of added elements

    # Rolling lists for all hotspot coordinates
    self.lat_lon_elements = np.empty((0,2))
    self.enu_xs = []
    self.enu_ys = []

    self.create_subscription(String, "/hot_spots", self.add_elements)
    self.pub = self.create_publisher(Float32, '/fire_gps_pin', 10)

  def send_command(self, msg):
    output = Float32()
    output.latitude = msg.latitude
    output.longitude = msg.longitude
    self.pub.publish(output)


  def hotspot_callback(self, msg):
    """
    Parse the incoming JSON and accumulate hotspots.
    """
    try:
        data = json.loads(msg.data)
        hotspots = data.get("hotspots", [])
        for h in hotspots:
            lat, lon, ex, ey, w = h
            new_row = np.array([[lat, lon]])
            self.lat_lon_elements = np.vstack((self.lat_lon_elements, new_row))

        self.get_logger().info(f"Received {len(hotspots)} hotspots (total={len(self.lats)}).")
        add_elements(self.lat_lon_elements);
    except Exception as e:
        self.get_logger().error(f"Failed to parse hotspot JSON: {e}")


  def add_elements(self):
    def add_elements(self, elements):
    """
    Add elements to the clustering object.
    :param elements: A list or NumPy array of 2D points to add.
    """
    elements = np.array(elements)
    for element in elements:
      self.elements = np.vstack([self.elements, element])
      if self.centroids.shape[0] == 0:
        # If no centroids exist, add the first one
        self.centroids = np.array([element])
      else:
        # Check distances to all centroids
        distances = np.linalg.norm(self.centroids - element, axis=1)
        if np.all(distances > self.max_radius):
          # Add a new centroid if no centroids are within max_radius
          self.centroids = np.vstack([self.centroids, element])

    # Assign each element to the nearest centroid
    self.labels = []
    for element in self.elements:
      distances = np.linalg.norm(self.centroids - element, axis=1)
      self.labels.append(np.argmin(distances))
    self.labels = np.array(self.labels)

    # Update centroids as the mean of their assigned points
    new_centroids = []
    for i in range(self.centroids.shape[0]):
      points = self.elements[self.labels == i]
      if len(points) > 0:
        new_centroids.append(points.mean(axis=0))
      else:
        new_centroids.append(self.centroids[i])  # Keep unchanged if no points assigned

    self.centroids = np.array(new_centroids)

  def convert_to_coords(self):
    """
    Convert the current centroids to a list of Coords messages.
    :return: A list of Coords messages.
    """
    return [Coords(x=self.centroids[i, 0], y=self.centroids[i, 1]) for i in range(self.centroids.shape[0])]

  def get_centroids(self):
    """
    Get the current centroids with at least min_size elements.
    :return: A NumPy array of the current centroids that meet the min_size criterion.
    """
    if self.centroids.shape[0] == 0:
      return np.empty((0, 2))  # No centroids available

    # Count points assigned to each centroid
    counts = np.bincount(self.labels, minlength=self.centroids.shape[0])
    filtered_centroids = self.centroids[counts >= self.min_size]

    self.send_command(filtered_centroids)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicKMeans(max_radius=0.5, min_size=1) # Adjust max_radius and min_size as needed
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
