import rclpy
from rclpy.node import Node
import numpy as np
from detection.msg import Coords, Clusters


class DynamicKMeans(Node):
  def __init__(self, max_radius, min_size=1):
    """
    Initialize the dynamic k-means object.
    :param max_radius: The maximum radius to consider a point close to a centroid.
    :param min_size: Minimum number of elements required for a centroid to be returned.
    """
    self.max_radius = max_radius
    self.min_size = min_size
    self.centroids = np.empty((0, 2))  # Start with no centroids
    self.elements = np.empty((0, 2))  # Start with no elements
    self.labels = []  # Store labels for the most recent batch of added elements

    self.create_subscription(Pixels, '/hot_spots', self.add_elements) #FIXME: Subscribe to correct topic
    self.pub = self.create_publisher(Clusters, '/clusters', 10); #FIXME: find out what to publish

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

    return filtered_centroids

def main(args=None):
    rclpy.init(args=args)
    node = Node('kmeans')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
