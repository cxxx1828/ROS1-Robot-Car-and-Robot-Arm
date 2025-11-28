#!/usr/bin/python

import rospy

import array
from fcntl import ioctl

from sensor_msgs.msg import Joy

class JoyType:
	DEFAULT = 1
	REDRAGON = 2
	XBOX = 3


def fill(fill_with, N):
	v = []
	for i in range(N):
		v.append(fill_with)
	return v

class JoyRemapper:
	def __init__(self):
		#TODO
		#rospy.logwarn('JOY_REMAPPER warn')
		#rospy.logdebug('JOY_REMAPPER debug')
		# Get joy type.
		self.joy_type = JoyType.DEFAULT
		joy_dev = rospy.get_param('~joy_dev', None)
		if joy_dev:
			#rospy.logdebug('joy_dev = {}'.format(joy_dev))
			with open(joy_dev, 'rb') as f:
				# Get the device name.
				buf = array.array('B', [0] * 64)
				def JSIOCGNAME(len):
					return 0x80006a13 + (0x10000 * len)
				if ioctl(f, JSIOCGNAME(len(buf)), buf) < 0:
					js_name = "Unknown"
				else:
					js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
				rospy.loginfo('Device name = {}'.format(js_name))
				if js_name == 'Microsoft X-Box 360 pad':
					self.joy_type = JoyType.XBOX
				elif js_name == 'ShanWan PS3/PC Wired GamePad':
					self.joy_type = JoyType.REDRAGON
					#self.joy_type = JoyType.DEFAULT
		rospy.loginfo('self.joy_type = {}'.format(self.joy_type))

		self.joy_src_sub = rospy.Subscriber("joy_src", Joy, self.on_joy)

		self.joy_dst_pub = rospy.Publisher(
			'joy_dst',
			Joy,
			queue_size = 1
		)



	def on_joy(self, ij):
		if self.joy_type == JoyType.DEFAULT:
			self.joy_dst_pub.publish(ij)
		elif self.joy_type == JoyType.REDRAGON:
			oj = Joy()

			oj.axes = fill(0.0, 8)
			oj.buttons = fill(0, 13)

			oj.axes[0] = ij.axes[0]
			oj.axes[1] = ij.axes[1]
			oj.axes[3] = ij.axes[2]
			oj.axes[4] = ij.axes[3]
			oj.axes[6] = ij.axes[4]
			oj.axes[7] = ij.axes[5]

			for i in range(0, len(ij.buttons)):
				oj.buttons[i] = ij.buttons[i]

			self.joy_dst_pub.publish(oj)

		elif self.joy_type == JoyType.XBOX:
			oj = Joy()

			oj.axes = fill(0.0, 8)
			oj.buttons = fill(0, 13)

			for i in [0, 1, 3, 4, 6, 7]:
				oj.axes[i] = ij.axes[i]
			#for i in [2, 5]:
			#	oj.axes[i] = ij.axes[i]

			buttons_mapping = [3, 1, 0, 2, 4, 5, -1, -1, 6, 7, 9, 10, 8]
			for oi, ii in enumerate(buttons_mapping):
				if ii != -1:
					oj.buttons[oi] = ij.buttons[ii]

			# LB
			if ij.axes[2] < 0:
				oj.buttons[6] = 1
			else:
				oj.buttons[6] = 0
			# RB
			if ij.axes[5] < 0:
				oj.buttons[7] = 1
			else:
				oj.buttons[7] = 0

			self.joy_dst_pub.publish(oj)



if __name__ == '__main__':
	rospy.init_node('joy_remapper')
	remapper = JoyRemapper()
	rospy.spin()
