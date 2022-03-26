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
from sensor_msgs.msg import Joy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class Republisher(Node):

    def __init__(self):
        super().__init__('republisher')

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)

        self.publisher = [
            self.create_publisher(Float32, 'whlspd1', 10),
            self.create_publisher(Float32, 'whlspd2', 10),
            self.create_publisher(Float32, 'whlspd3', 10),
            self.create_publisher(Float32, 'whlspd4', 10),
            ]

        self.buttonlist = [
          'Square',
          'Cross',
          'Circle',
          'Triangle',
          'L1',
          'R1',
          'L2',
          'R2',
          'Share',
          'Option',
          'L3',
          'R3',
          'PlayStation',
          'Touchpad'
        ]

        self.axeslist = [
          'left_x',
          'left_y',
          'right_x',
          'L2_axes',
          'R2_axes'
        ]

        # Speed options
            # Default initial speed state
        self.speed_state = 2
        self.rtspd_state = 2
            # Speed options : plane speed in m/s | rotation speed in rad/s
        self.speed_option = [0.25, 0.5, 1, 2, 2.7]
        self.rtspd_option = [np.pi/6, np.pi/4, np.pi/2, np.pi, np.pi * 3/2]

        # Pressed buttons
            # for press_list:
            #   1: A new press instance
            #  -1: Button is released
            #   0: Button follows previous state
        self.last_state = [0] * len(self.buttonlist)
        self.press_list = [0] * len(self.buttonlist)

        # RPM Calculation variables
        self.finalRPM = np.zeros(4)
        self.finalRPM1 = self.finalRPM2 = self.finalRPM3 = self.finalRPM4 = Float32()
        self.finalRPMarray = [
            self.finalRPM1,
            self.finalRPM2,
            self.finalRPM3,
            self.finalRPM4
        ]
            # Angle of the car where CCW +ve
        self.theta = float()
            # Target V_x, V_y, W_z
        self.targetSpeed = np.zeros(3)
            # Jacobian Matrix
        self.jacob = np.empty([3, 4])
            # Radii of the car (in meters)
                # [0]: Wheel radius
                # [1]: Car radius
        self.radii = np.array([0.126, 2])


    def listener_callback(self, msg):

        # To detect new button press instance
        for i in range(len(self.buttonlist)):
            self.press_list[i] = msg.buttons[i] - self.last_state[i]
        self.last_state = msg.buttons


      	# To control Speed states with R1/L1 (R2 hold toggle)
        if self.press_list[5] == 1:    # R1 = press_list[5]
            if msg.buttons[7] == 1:    # R2 hold
                self.rtspd_state = min(self.rtspd_state + 1, len(self.rtspd_option) - 1)
            else:
                self.speed_state = min(self.speed_state + 1, len(self.speed_option) - 1)

        if self.press_list[4] == 1:    # L1 = press_list[4]
            if msg.buttons[7] == 1:    # R2 hold
                self.rtspd_state = max(0, self.rtspd_state - 1)
            else:
                self.speed_state = max(0, self.speed_state - 1)


        # To calculate the final target speed to be published
            # Get V_x, V_y, W_z
        self.targetSpeed = np.array([
            msg.axes[0] * self.speed_option[self.speed_state],
            msg.axes[1] * self.speed_option[self.speed_state],
            msg.axes[2] * self.rtspd_option[self.rtspd_state]
            ])

            # Angle of the car
        self.theta = 0.0

            # Jacobian Matrix
        self.jacob = np.array([
            [  np.sin(self.theta), -np.cos(self.theta), self.radii[1] ],
            [  np.cos(self.theta),  np.sin(self.theta), self.radii[1] ],
            [ -np.sin(self.theta),  np.cos(self.theta), self.radii[1] ],
            [ -np.cos(self.theta), -np.sin(self.theta), self.radii[1] ]
        ])

            # Finalize RPM
        self.finalRPM = (1 / self.radii[0]) * np.matmul(self.jacob, self.targetSpeed)
        self.finalRPM *= 60 / (2 * np.pi)

            # Publish
        for i in range(len(self.finalRPM)):
            self.finalRPMarray[i].data = float(self.finalRPM[i])
            self.publisher[i].publish(self.finalRPMarray[i])


def main(args=None):
    rclpy.init(args=args)

    republisher = Republisher()
    rclpy.spin(republisher)

    # Destroy the node explicitly
    republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()