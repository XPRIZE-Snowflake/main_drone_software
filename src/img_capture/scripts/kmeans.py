#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import json
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class DynamicKMeans(Node):
    def __init__(self):
        """
        Initialize the dynamic k-means object.
        :param max_radius: The maximum radius to consider a point close to a centroid.
        :param min_size: Minimum number of elements required for a centroid to be returned.
        """
        super().__init__('dynamic_kmeans')
        self.get_logger().info("Dynamic K Means Node started.\n")

        self.declare_parameter('max_radius', 10.0)  # Default: 10.0
        self.declare_parameter('min_size', 5)       # Default: 5

        self.max_radius = self.get_parameter('max_radius').value
        self.min_size = self.get_parameter('min_size').value

        self.centroids = np.empty((0, 2))  # Start with no centroids
        self.elements = np.empty((0, 2))  # Start with no elements
        self.labels = np.array([])  # Store labels for the most recent batch of added elements
        
        self.lat_lon_elements = np.empty((0, 2))  # Rolling list of all received coordinates
        
        self.create_subscription(String, "hot_spots", self.hotspot_callback, 10)
        self.pub = self.create_publisher(NavSatFix, 'fire_gps_pin', 10)
    
    def send_command(self, centroid):
        """
        Publish the centroid coordinates as a GPS message.
        """
        msg = NavSatFix()
        msg.latitude = centroid[0]
        msg.longitude = centroid[1]
        self.pub.publish(msg)
        self.get_logger().debug(f"Published GPS coordinate: lat={centroid[0]}, lon={centroid[1]}")

    def hotspot_callback(self, msg):
        """
        Parse the incoming JSON and accumulate hotspots.
        """
        try:
            data = json.loads(msg.data)
            hotspots = data.get("hotspots", [])
            
            for h in hotspots:
                lat, lon, *_ = h  # Extract latitude and longitude
                new_row = np.array([[lat, lon]])
                self.lat_lon_elements = np.vstack((self.lat_lon_elements, new_row))
            
            self.get_logger().debug(f"Received {len(hotspots)} new hotspots (total={len(self.lat_lon_elements)}).")
            self.add_elements(self.lat_lon_elements)
        except Exception as e:
            self.get_logger().error(f"Failed to parse hotspot JSON: {e}")

    def add_elements(self, elements):
        """
        Add elements to the clustering object.
        :param elements: A list or NumPy array of 2D points to add.
        """
        elements = np.array(elements)
        for element in elements:
            self.elements = np.vstack([self.elements, element])
            if self.centroids.shape[0] == 0:
                self.centroids = np.array([element])
            else:
                distances = np.linalg.norm(self.centroids - element, axis=1)
                if np.all(distances > self.max_radius):
                    self.centroids = np.vstack([self.centroids, element])
        
        # Assign each element to the nearest centroid
        self.labels = np.array([np.argmin(np.linalg.norm(self.centroids - e, axis=1)) for e in self.elements])
        
        # Update centroids as the mean of their assigned points
        new_centroids = []
        for i in range(self.centroids.shape[0]):
            points = self.elements[self.labels == i]
            if len(points) > 0:
                new_centroids.append(points.mean(axis=0))
            else:
                new_centroids.append(self.centroids[i])  # Keep unchanged if no points assigned
        
        self.centroids = np.array(new_centroids)
        self.get_logger().debug(f"Updated centroids: {self.centroids}")
        self.get_centroids()

    def get_centroids(self):
        """
        Get the current centroids with at least min_size elements.
        """
        if self.centroids.shape[0] == 0:
            return np.empty((0, 2))
        
        # Count assigned points to each centroid
        counts = np.bincount(self.labels, minlength=self.centroids.shape[0])
        valid_indices = np.where(counts >= self.min_size)[0]
        filtered_centroids = self.centroids[valid_indices]
        
        for centroid in filtered_centroids:
            self.send_command(centroid)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicKMeans()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()