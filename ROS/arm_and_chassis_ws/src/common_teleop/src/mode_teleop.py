#!/usr/bin/python

import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

import subprocess

def fill(fill_with, N):
	v = []
	for i in range(N):
		v.append(fill_with)
	return v

class ModeTeleop:
	def __init__(self):
		self.first_time = True
		
		self.modes = ['jog', 'servo']
		en_chassis = rospy.get_param('~en_chassis')
		if en_chassis:
			self.modes.append('chassis')
		
		self.mode = self.modes[0]
		self.arm_motors_en = True
		self.chassis_motors_en = True
		
		self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)
		
		self.arm_motors_en_pub = rospy.Publisher(
			'arm/motors_en',
			Bool,
			queue_size = 1
		)
		self.chassis_motors_en_pub = rospy.Publisher(
			'chassis/motors_en',
			Bool,
			queue_size = 1
		)
		
		rospy.loginfo('''

Mode Teleop:
	
	Buttons:
		Select: change mode {modes}
		
		Start: Start/Stop motors
		
		'''.format(modes = self.modes))

	def publish_motors_en(self):
		msg = Bool()
		msg.data = self.arm_motors_en
		self.arm_motors_en_pub.publish(msg)
		msg.data = self.chassis_motors_en
		self.chassis_motors_en_pub.publish(msg)
	

	def on_joy(self, data):
		if self.first_time:
			self.first_time = False
			self.prev_buttons = data.buttons
			self.buttons_re = fill(False, len(data.buttons))
			return
		
		for i in range(len(self.buttons_re)):
			self.buttons_re[i] = not self.prev_buttons[i] and data.buttons[i]
		self.prev_buttons = data.buttons
		
		
		if self.buttons_re[8]: # Select
			i = self.modes.index(self.mode) + 1
			assert(i <= len(self.modes))
			if i == len(self.modes):
				i = 0
			self.mode = self.modes[i]
			rospy.loginfo('Current mode: {}'.format(self.mode))
			if self.mode == 'chassis':
				self.arm_motors_en = False
				self.chassis_motors_en = True
				self.publish_motors_en()
			else:
				self.arm_motors_en = True
				self.chassis_motors_en = False
				self.publish_motors_en()
				if self.mode == 'jog':
					ctrl = 'jog'
				elif self.mode == 'servo':
					ctrl = 'servo'
					#ctrl = 'traj_servo'
				cmd = 'rosrun common_teleop change_controller.py --silent'.split()
				cmd.append(ctrl)
				subprocess.run(cmd)
			
			
		if self.buttons_re[9]: # Start
			if self.arm_motors_en or self.chassis_motors_en:
				rospy.loginfo('Motors disabled')
				self.arm_motors_en = False
				self.chassis_motors_en = False
			else:
				if self.mode == 'chassis':
					rospy.loginfo('Chassis motors enabled')
					self.arm_motors_en = False
					self.chassis_motors_en = True
				else:
					rospy.loginfo('Arm motors enabled')
					self.arm_motors_en = True
					self.chassis_motors_en = False
			self.publish_motors_en()
		

if __name__ == '__main__':
	rospy.init_node('mode_teleop')
	controller = ModeTeleop()
	rospy.spin()
