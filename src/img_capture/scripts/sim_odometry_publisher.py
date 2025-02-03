import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Vector3
import numpy as np
import os

class SimulatedOdometryPublisher(Node):
    def __init__(self):
        super().__init__('simulated_odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/simulated_odometry', 10)
        self.timer = self.create_timer(1.0, self.publish_odometry)
        self.data = self.load_npy_data()
        self.current_index = 0

    def load_npy_data(self):
        file_path = self.declare_parameter('npy_file_path', '').value
        if not file_path:
            self.get_logger().error('NPY file path not specified')
            return None
        if not os.path.exists(file_path):
            self.get_logger().error(f'NPY file not found: {file_path}')
            return None
        return np.load(file_path)

    def publish_odometry(self):
        if self.data is None or self.current_index >= len(self.data):
            return

        pose_data = self.data[self.current_index]
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'

        # Assuming the order of fields in the NumPy array is:
        # timestamp, latitude, longitude, altitude, pitch, roll, yaw
        msg.pose.pose.position = Point(x=float(pose_data[2]),  # longitude
                                       y=float(pose_data[1]),  # latitude
                                       z=float(pose_data[3]))  # altitude

        # Set linear and angular velocities to zero (or calculate if available)
        msg.twist.twist = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0),
                                angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.publisher_.publish(msg)
        
        # Log raw pitch, roll, and yaw values instead of quaternion
        self.get_logger().info(f'Published odometry data: '
                               f'Timestamp: {pose_data[0]}, '
                               f'Position: ({pose_data[2]}, {pose_data[1]}, {pose_data[3]}), '
                               f'Orientation: Pitch={pose_data[4]}, Roll={pose_data[5]}, Yaw={pose_data[6]}')

        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
