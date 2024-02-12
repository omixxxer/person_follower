# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        self.target_distance = 0.1
        self.wall_distance = 0.1

    def listener_callback(self, input_msg):
        ranges = input_msg.ranges

        # Filtro de mediana para reducir el ruido en los datos del escáner láser
        filtered_ranges = self.median_filter(ranges)

        # Encuentra el punto objetivo más cercano
        self.target_point = self.find_closest_point(filtered_ranges, input_msg.angle_min, input_msg.angle_increment)

        # Calcula la distancia al punto objetivo
        self.target_distance = np.sqrt(self.target_point.x ** 2 + self.target_point.y ** 2)

        # Calcula la distancia a la pared más cercana
        self.wall_distance = min(filtered_ranges)

        # Calcula la velocidad lineal y angular
        vx = 0.1  # Velocidad lineal inicialmente cero
        wz = 0.0  # Velocidad angular inicialmente cero

        # Define los umbrales para la navegación
        person_distance_threshold = 0.1  # Distancia mínima para seguir a la persona
        min_wall_distance = 0.2  # Distancia mínima para mantenerse de las paredes

        print("Target Distance:", self.target_distance)

        # Si la distancia al objetivo está por debajo del umbral de seguimiento de la persona
        if self.target_distance < person_distance_threshold:
            # Calcula la velocidad angular para seguir al objetivo, ajustada por la distancia a la pared
            desired_wz = np.arctan2(self.target_point.y, self.target_point.x)
            if self.wall_distance < min_wall_distance:
                # Si estamos demasiado cerca de la pared, ajusta la velocidad angular para alejarse de ella
                wz = np.sign(desired_wz) * 0.5
            else:
                # Si no, sigue la dirección del punto objetivo
                wz = desired_wz
            print("Following person with angular velocity:", wz)

        # Publica las velocidades con suavizado
        output_msg = Twist()
        output_msg.linear.x = vx
        output_msg.angular.z = wz
        self.publisher_.publish(output_msg)

    def median_filter(self, ranges, window_size=5):
        # Implementa un filtro de mediana para suavizar los datos del escáner láser
        filtered_ranges = []
        half_window = window_size // 2
        for i in range(len(ranges)):
            start = max(0, i - half_window)
            end = min(len(ranges), i + half_window + 1)
            filtered_ranges.append(np.median(ranges[start:end]))
        return filtered_ranges

    def find_closest_point(self, ranges, angle_min, angle_increment):
        # Encuentra el punto más cercano en el escáner láser
        min_range = min(ranges)
        min_index = ranges.index(min_range)
        angle = angle_min + min_index * angle_increment
        x = min_range * np.cos(angle)
        y = min_range * np.sin(angle)
        point = Point()
        point.x = x
        point.y = y
        return point

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

