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
        self.speed_state = 2
        self.rtspd_state = 2
        self.speed_option = [0.1, 0.2, 0.4, 0.6, 0.8]
        self.rtspd_option = [np.pi/64, np.pi/32, np.pi/16, np.pi/8, np.pi/4]

        # Pressed buttons
            # for press_list:
            #   1: A new press instance
            #  -1: Button is released
            #   0: Button follows previous state
        self.last_state = [0] * len(self.buttonlist)
        self.press_list = [0] * len(self.buttonlist)

        # RPM Calculation variables
            # Final RPM data to be published to Arduino
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
        self.radii = np.array([0.05, 0.1])

    # when new instance:
    def listener_callback(self, msg):
        # To detect new button press instance
        for i in range(len(self.buttonlist)):
            self.press_list[i] = msg.buttons[i] - self.last_state[i]
        self.last_state = msg.buttons

      	# speed state control by R1 and L1
            # R1 speed state increase | L1 speed state decrease
            # R1 = press_list[5] | L1 = press_list[4]
            
#        if self.press_list[5] == 1:
#           if msg.buttons[7] & self.press_list[5] == 1:
#              if self.rtspd_state < len(self.rtspd_option) - 1:
#                 self.rtspd_state += 1
#              elif self.rtspd_state == len(self.rtspd_option) - 1 :
#                 self.rtspd_state = 0
#        else:
#           if self.speed_state < len(self.speed_option) - 1:
#                 self.speed_state += 1
#           elif self.speed_state == len(self.speed_option) - 1 :
#                 self.speed_state = 0       
#                 
#        if self.press_list[4] == 1:
#           if msg.buttons[7] & self.press_list[4] == 1:
#              if self.rtspd_state > 0:
#                 self.rtspd_state -= 1
#              elif self.rtspd_state == 0:
#                 self.rtspd_state = len(self.rtspd_option) - 1
#        else:
#           if self.speed_state > 0:
#                 self.speed_state -= 1
#           elif self.speed_state == 0 :
#                 self.speed_state = len(self.speed_option) - 1
                 
        if self.press_list[5] == 1:
            if msg.buttons[7] == 1:
                self.rtspd_state = min(self.rtspd_state + 1, len(self.rtspd_option) - 1)
            else:
                self.speed_state = min(self.speed_state + 1, len(self.speed_option) - 1)
                
        if self.press_list[4] == 1:
            if msg.buttons[7] == 1:
                self.rtspd_state = max(0, self.rtspd_state - 1)
            else:
                self.speed_state = max(0, self.speed_state - 1)
        
            
        self.targetSpeed = np.array([
            msg.axes[0] * self.speed_option[self.speed_state],
            msg.axes[1] * self.speed_option[self.speed_state],
            msg.axes[2] * self.rtspd_option[self.rtspd_state]
            ])

        self.theta = 0.0

        self.jacob = np.array([
            [  np.sin(self.theta), -np.cos(self.theta), self.radii[1] ],
            [  np.cos(self.theta),  np.sin(self.theta), self.radii[1] ],
            [ -np.sin(self.theta),  np.cos(self.theta), self.radii[1] ],
            [ -np.cos(self.theta), -np.sin(self.theta), self.radii[1] ]
        ])

        self.finalRPM = (1 / self.radii[0]) * np.matmul(self.jacob, self.targetSpeed)
        self.finalRPM *= 60 / (2 * np.pi)
        
        for i in range(len(self.finalRPM)):
            self.finalRPMarray[i].data = float(self.finalRPM[i])
            self.publisher[i].publish(self.finalRPMarray[i])


        # Targetspeed for ID 1,2,3,4 respectivly
        # self.targetspeed = [1,1,1,1] * self.speed_option[self.speed_state]

      	# Normalize analog joystick and convert to wheel-pair coefficient
        # 14 = self.speed_option[self.speed_state] * coffe.imag | 23 = self.speed_option[self.speed_state] * coffe.real
        # Left Stick X-Axis = msg.axes[0] | Left Stick Y-Axis = msg.axes[1] | Right Stick X-Axis = msg.axes[2]
				# Move & Rotation
        # if ((msg.axes[0] != 0) & (msg.axes[1] != 0)) | (msg.axes[2] != 0):
	    #     self.newAngle = np.angle(msg.axes[0] + msg.axes[1] * j) - pi/4
  		#   	self.coffe = np.exp(j * self.newAngle)
    	# 		self.alpha_beta = [self.coffe.imag, self.coffe.real]
        #   self.rotation =  self.speed_option[self.speed_state] * msg.axes[2] * -1
        #   self.mix1 = self.speed_option[self.speed_state] * self.coffe.imag + self.rotation
        #   self.mix2 = self.speed_option[self.speed_state] * self.coffe.real + self.rotation * -1
        #   self.mix3 = self.speed_option[self.speed_state] * self.coffe.imag + self.rotation
        #   self.mix4 = self.speed_option[self.speed_state] * self.coffe.real + self.rotation * -1
        #   self.targetspeed[0] = self.mix1
        #   self.targetspeed[1] = self.mix2
        #   self.targetspeed[2] = self.mix3
        #   self.targetspeed[3] = self.mix4
        #   self.publisher.publish(self.targetspeed)


def main(args=None):
    rclpy.init(args=args)

    republisher = Republisher()
    rclpy.spin(republisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()