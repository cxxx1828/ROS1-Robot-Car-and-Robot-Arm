#!/usr/bin/env python

import argparse

parser = argparse.ArgumentParser(
	prog = 'conv.py',
	description = '''
		Convert between degress, radians,
		wide duty, servo motor narrow duty
	'''
)
formats = [
	'deg',
	'rad',
	'wide_duty',
	'servo_duty'
]
parser.add_argument(
	'in_fmt',
	metavar = 'in_fmt',
	choices = formats,
	help = f'Format of input (possible: {formats})'
)
parser.add_argument(
	'out_fmt',
	metavar = 'out_fmt',
	choices = formats,
	help = f'Format of output (possible: {formats})'
)
parser.add_argument(
	'in_val',
	metavar = 'in_val',
	type = float,
	help = f'Input value to convert'
)
args = parser.parse_args()

if False:
	print(args.in_fmt)
	print(args.out_fmt)
	print(args.in_val)

from cmath import pi

def rad2deg(rad):
	return 180/pi*rad
def deg2rad(deg):
	return pi/180*deg

# deg is intermediate format.

# Key is in_fmt, value is fun to convert to deg.
in_conv = {
	'deg'        : lambda deg: deg,
	'rad'        : rad2deg,
	'wide_duty'  : lambda val: ((duty) - 100)*(180.0/800) - 90,
	'servo_duty' : lambda val: ((duty) - 25)*(180.0/100) - 90,
}

# Key is out_fmt, value is fun to convert from deg.
out_conv = {
	'deg'        : lambda deg: deg,
	'rad'        : deg2rad,
	'wide_duty'  : lambda deg: (deg + 90)*(800/180.0) + 100,
	'servo_duty' : lambda deg: (deg + 90)*(100/180.0) + 25,
}

mid_deg = in_conv[args.in_fmt](args.in_val)
out_val = out_conv[args.out_fmt](mid_deg)
print(out_val)
