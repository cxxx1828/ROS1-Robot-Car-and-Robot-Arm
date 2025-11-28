#!/usr/bin/python

import rospy

from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import JointState

import math

import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from common.jog_teleop import *

SPEED = 0.03

class JogPosTeleop(JogTeleop):
	def __init__(self):
		super().__init__()
		
		
		self.pos_pub = rospy.Publisher(
			'jog_pos_cmd',
			Float64MultiArray,
			queue_size = 1
		)
		self.motors_freewheel_pub = rospy.Publisher(
			'motors_freewheel',
			Bool,
			queue_size = 1
		)
		
		self.joints_fb_sub = rospy.Subscriber(
			"joints_fb",
			JointState,
			self.on_joints_fb
		)
		
		self.pos_cmd = fill(0.0, self.n_joints)
		self.pos_fb = fill(0.0, self.n_joints)
		
		self.print_help('''
		2	Toggle motors freewheel
		
		''')

	def on_joy(self, data):
		super().on_joy(data)
		
		if self.buttons_re[1]: # 2
			self.motors_freewheel = not self.motors_freewheel
			if self.motors_freewheel:
				rospy.loginfo('Motors freewheel')
			else:
				rospy.loginfo('Motors powered')
			msg = Bool()
			msg.data = self.motors_freewheel
			self.motors_freewheel_pub.publish(msg)
		
		
		if abs(self.axis) >= 0.01:
			id = self.active_motor
			
			p = self.pos_cmd[id]
			d = SPEED*self.axis
			#rospy.logwarn('d = {}'.format(d))
			#rospy.logwarn('p = {}'.format(p))
			p += d
			if p > self.pos_max[id]:
				p = self.pos_max[id]
			elif p < self.pos_min[id]:
				p = self.pos_min[id]
			self.pos_cmd[id] = p
		else:
			# When no input, set to fb
			#FIXME Do not set to pos_fb until velocity come to 0.
			#self.pos_cmd[0:self.n_joints] = self.pos_fb[0:self.n_joints]
			pass
		
		msg = Float64MultiArray()
		msg.data = self.pos_cmd
		self.pos_pub.publish(msg)
		
	def on_joints_fb(self, data):
		#rospy.logwarn('data.position[0] = {}'.format(data.position[0]))
		self.pos_fb[0:self.n_joints] = data.position[0:self.n_joints]
		

if __name__ == '__main__':
	rospy.init_node('jog_teleop')
	controller = JogPosTeleop()
	rospy.spin()
