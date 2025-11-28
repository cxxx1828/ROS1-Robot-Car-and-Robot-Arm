#!/usr/bin/python

import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

import moveit_commander

import subprocess

def fill(fill_with, N):
	v = []
	for i in range(N):
		v.append(fill_with)
	return v

class EventerTeleop:
	def __init__(self):
		self.first_time = True
		
		self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)
		
		self.event_re_pub = rospy.Publisher(
			'event_re',
			Bool,
			queue_size = 1
		)
		self.event_hold_pub = rospy.Publisher(
			'event_hold',
			Bool,
			queue_size = 1
		)
		self.event_merged_pub = rospy.Publisher(
			'event_merged',
			Bool,
			queue_size = 1
		)
		
		rospy.loginfo('''

Eventer Teleop:
	
	Buttons:
		3: RE - trigger just one event
		4: HOLD - trigger multiple events while holding
		
		''')

	def on_joy(self, data):
		if self.first_time:
			self.first_time = False
			self.prev_buttons = data.buttons
			self.buttons_re = fill(False, len(data.buttons))
			return
		
		for i in range(len(self.buttons_re)):
			self.buttons_re[i] = not self.prev_buttons[i] and data.buttons[i]
		self.prev_buttons = data.buttons

			
		re = self.buttons_re[2] # 3
		msg = Bool()
		msg.data = re
		self.event_re_pub.publish(msg)
		
		
		hold = data.buttons[3] # 4
		msg = Bool()
		msg.data = hold
		self.event_hold_pub.publish(msg)
		
		msg = Bool()
		msg.data = re or hold
		self.event_merged_pub.publish(msg)
		

if __name__ == '__main__':
	rospy.init_node('eventer_teleop')
	controller = EventerTeleop()
	rospy.spin()
