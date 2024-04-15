import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
import numpy as np

class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.target_point = Point()
        self.target_distance = 0.2
        self.wall_distance = 0.0

        # Define thresholds
        self.PERSON_DISTANCE_THRESHOLD = 0.4
        self.MIN_WALL_DISTANCE = 0.1

        # Define linear and angular velocities
        self.max_linear_velocity = 0.3
        self.max_angular_velocity = 1.0

    def listener_callback(self, input_msg):
        ranges = input_msg.ranges

        # Median filter to reduce noise
        filtered_ranges = self.median_filter(ranges)

        # Find closest point
        min_range_idx = np.argmin(filtered_ranges)
        min_range = filtered_ranges[min_range_idx]
        angle = input_msg.angle_min + min_range_idx * input_msg.angle_increment
        x = min_range * np.cos(angle)
        y = min_range * np.sin(angle)

        # Update target point and distances
        self.target_point.x = x
        self.target_point.y = y
        self.target_distance = min_range
        self.wall_distance = np.min(filtered_ranges)

        # Calculate desired angular velocity to follow the target
        desired_wz = np.arctan2(self.target_point.y, self.target_point.x)

        # Limit angular velocity
        self.limit_angular_velocity(desired_wz)

        # Adjust linear velocity based on target distance and wall distance
        self.adjust_linear_velocity()

        # Publish velocities
        output_msg = Twist()
        output_msg.linear.x = self.vx
        output_msg.angular.z = self.wz
        self.publisher_.publish(output_msg)

    def median_filter(self, ranges, window_size=5):
        half_window = window_size // 2
        filtered_ranges = np.zeros_like(ranges)
        for i in range(len(ranges)):
            start = max(0, i - half_window)
            end = min(len(ranges), i + half_window + 1)
            filtered_ranges[i] = np.median(ranges[start:end])
        return filtered_ranges

    def limit_angular_velocity(self, desired_wz):
        self.wz = np.clip(desired_wz, -self.max_angular_velocity, self.max_angular_velocity)

    def adjust_linear_velocity(self):
        if self.target_distance > self.PERSON_DISTANCE_THRESHOLD:
            self.vx = min(self.max_linear_velocity, self.target_distance - self.PERSON_DISTANCE_THRESHOLD)
        else:
            self.vx = 0.0

        if self.wall_distance < self.MIN_WALL_DISTANCE:
            self.wz += np.sign(self.wz) * 0.5  # Turn away from the wall

	# Turn away from the wall
        self.wz += np.sign(self.wz) * 1.5  

        # Limit angular velocity to prevent spinning too fast
        self.wz = np.clip(self.wz, self.max_angular_velocity, -self.max_angular_velocity)
        
def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()       

