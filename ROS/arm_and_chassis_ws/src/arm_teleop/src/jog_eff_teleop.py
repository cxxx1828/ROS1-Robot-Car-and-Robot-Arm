#!/usr/bin/python

import rospy

from std_msgs.msg import Bool, Float64MultiArray

import math

import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from common.jog_teleop import *

# Range: [0, 1]
#speeds = [0.4, 1.0, 0.6, 1.0, 0.5, 0.8]
speeds = [0.4, 0.5, 0.5, 0.7, 0.8, 0.8]

class JogEffTeleop(JogTeleop):
	def __init__(self):
		super().__init__()
		
		self.motors_freewheel = False
		
		self.eff_pub = rospy.Publisher(
			'jog_eff_cmd',
			Float64MultiArray,
			queue_size = 1
		)

		self.calib_pub = rospy.Publisher(
			'calib',
			Bool,
			queue_size = 1
		)
		
		self.print_help('''
		2	Calib all motors
		
		''')

	def on_joy(self, data):
		super().on_joy(data)
		
		if self.buttons_re[1]: # 2
			msg = Bool()
			msg.data = True
			self.calib_pub.publish(msg)
		
		id = self.active_motor
		
		speed = speeds[id]
		e = self.axis*speed*100
		eff_cmd = fill(0, self.n_joints)
		eff_cmd[id] = e
		msg = Float64MultiArray()
		msg.data = eff_cmd
		self.eff_pub.publish(msg)


if __name__ == '__main__':
	rospy.init_node('jog_teleop')
	controller = JogEffTeleop()
	rospy.spin()
