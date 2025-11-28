#!/usr/bin/python

import rospy

from sensor_msgs.msg import Joy
from urdf_parser_py.urdf import URDF

import math

if False:
	import os
	import sys
	sys.path.append(os.path.dirname(os.path.realpath(__file__)))
	from common.utils import show

def fill(fill_with, N):
	v = []
	for i in range(N):
		v.append(fill_with)
	return v

class JogTeleop:
	def __init__(self):
		self.first_time = True

		robot = URDF.from_parameter_server()
		self.pos_min = []
		self.pos_max = []
		self.joint_names = []
		for name, joint in robot.joint_map.items():
			if name.startswith('joint'):
				self.joint_names.append(name)
				self.pos_min.append(joint.limit.lower)
				self.pos_max.append(joint.limit.upper)
		#print(self.joint_names)
		#print(self.pos_min)
		#print(self.pos_max)
		self.n_joints = len(self.joint_names)-1 # Last one is mirrored.

		self.active_motor = 0
		self.axis = 0.0

		self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)


	def print_help(self, additional):
		rospy.loginfo('''

Jog Teleop:

	L stick:
		U	+axis	CW		Red
		D	-axis	CCW		Blue

	Buttons:
		LT	active motor ++
		LB	active motor --

		''' + additional)

	def on_joy(self, data):
		if self.first_time:
			self.first_time = False
			self.prev_buttons = data.buttons
			self.buttons_re = fill(False, len(data.buttons))
			return

		for i in range(len(self.buttons_re)):
			self.buttons_re[i] = not self.prev_buttons[i] and data.buttons[i]
		self.prev_buttons = data.buttons


		def print_active_motor():
			rospy.loginfo(
				'Active motor: {} ({})'.format(
					self.active_motor,
					self.joint_names[self.active_motor]
				)
			)

		if self.buttons_re[4]: # LT
			# Move to next motor.
			self.active_motor += 1
			if self.active_motor == self.n_joints:
				self.active_motor = 0
			print_active_motor()
		elif self.buttons_re[6]: # LB
			self.active_motor -= 1
			if self.active_motor == -1:
				self.active_motor = self.n_joints-1
			print_active_motor()

		# L stick D- U+
		self.axis = data.axes[1] # In range [-1, 1]
