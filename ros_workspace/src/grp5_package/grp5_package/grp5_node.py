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
from std_msgs.msg import Int16
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
            self.create_publisher(Int16, 'whlspd1', 10),
            self.create_publisher(Int16, 'whlspd2', 10),
            self.create_publisher(Int16, 'whlspd3', 10),
            self.create_publisher(Int16, 'whlspd4', 10),
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
        self.speed_state = self.rtspd_state = 0
            # Speed options : plane speed in m/s | rotation speed in rad/s
        self.speed_option = [1500, 2500, 3300, 4200, 6000]
        self.rtspd_option = [800, 1400, 2000, 4000, 6000]

        # Pressed buttons
            # for press_list:
            #   1: A new press instance
            #  -1: Button is released
            #   0: Button follows previous state
        self.last_state = [0] * len(self.buttonlist)
        self.press_list = [0] * len(self.buttonlist)

        # RPM Calculation variables
        self.finalRPM = np.zeros(4)
        # TODO: Clean this up?
        self.finalRPM1 = self.finalRPM2 = self.finalRPM3 = self.finalRPM4 = Int16()
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
                # [0]: Wheel radius (measured 0.126)
                # [1]: Car radius   (measured 0.75)
        self.radii = np.array([1, 1])


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
                # ! For other potential implementations
        self.theta = 0.0

            # Jacobian Matrix
        self.jacob = np.array([
            [ -1, -1, -self.radii[1] ],
            [  1, -1, -self.radii[1] ],
            [  1,  1, -self.radii[1] ],
            [ -1,  1, -self.radii[1] ]
        ])

            # Finalize RPM
        self.finalRPM = np.matmul(self.jacob, self.targetSpeed) / self.radii[0]
        self.get_logger().info("")
        self.get_logger().info("finalRPM")
        self.get_logger().info("{}".format(self.finalRPM))

            # Publish
        for i in range(len(self.finalRPM)):
            self.finalRPMarray[i].data = int(self.finalRPM[i])
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
