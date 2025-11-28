#!/usr/bin/env python

import argparse
import subprocess
import rospy
from controller_manager_msgs.srv._ListControllers import ListControllers

def list_controllers():
	rospy.wait_for_service('/controller_manager/switch_controller')
	list_controllers = rospy.ServiceProxy(
		'/controller_manager/list_controllers',
		ListControllers
	)
	resp = list_controllers()
	mode_to_ctrls = {
		'jog' : [],
		'traj' : [],
		'servo' : [],
	}
	all_ctrls = []
	ctrl_to_state = {}
	for c in resp.controller:
		for mode in mode_to_ctrls.keys():
			if mode in c.name:
				mode_to_ctrls[mode].append(c.name)
				all_ctrls.append(c.name)
				ctrl_to_state[c.name] = c.state
				break
	active_mode = None
	for mode, controllers in mode_to_ctrls.items():
		for c in controllers:
			if all([ctrl_to_state[c] == 'running' for c in controllers]):
				active_mode = mode
				break
	return mode_to_ctrls, active_mode, ctrl_to_state, all_ctrls



def change_controller(mode, silent = True):
	mode_to_ctrls, _, _, all_ctrls = list_controllers()
	
	if mode not in mode_to_ctrls:
		print('Non supported controller "{}"'.format(mode))
	else:
		ctrls = mode_to_ctrls[mode]
	
	#TODO Python instead of call.
	cmd = 'rosservice call /controller_manager/switch_controller'.split()
	arg = '''
start_controllers: {for_start}
stop_controllers: {for_stop}
strictness: 1
start_asap: false
timeout: 0.0
'''.format(
		for_start = ctrls,
		for_stop = all_ctrls
	)
	cmd.append(arg)
	
	print('Changing to controller: {}'.format(mode))
	if not silent:
		print(subprocess.list2cmdline(cmd))
	subprocess.run(cmd) # Does not work: eat new lines.
	
	#TODO Traj servo
	if mode in ['servo', 'traj_servo']:
		cmd = 'rosservice call /servo_server/change_drift_dimensions'.split()
		arg = '''
drift_x_translation: false
drift_y_translation: false
drift_z_translation: false
drift_x_rotation: true
drift_y_rotation: false
drift_z_rotation: true
transform_jog_frame_to_drift_frame:
  translation: {x: 0.0, y: 0.0, z: 0.0}
  rotation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
'''
		cmd.append(arg)
		if not silent:
			print(subprocess.list2cmdline(cmd))
		subprocess.run(cmd) # Does not work: eat new lines.
	
	
if __name__ == "__main__":
	parser = argparse.ArgumentParser(
		description = __doc__
	)
	
	# Positional arguments
	mode_to_ctrls, _, _, _ = list_controllers()
	l = list(mode_to_ctrls.keys())
	parser.add_argument(
		dest = 'mode',
		metavar = 'mode',
		type = str,
		nargs = 1,
		choices = l,
		help = 'Controller to switch to ({})'.format(', '.join(l))
	)
	parser.add_argument(
		'--silent',
		action="store_true",
		help = 'Print command'
	)
	args = parser.parse_args()
	
	change_controller(
		args.mode[0],
		silent = args.silent
	)
	
