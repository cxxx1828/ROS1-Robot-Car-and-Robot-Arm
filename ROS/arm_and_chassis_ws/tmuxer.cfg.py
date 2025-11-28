
if not 'SIM' in globals():
	SIM = False
if not 'ARM' in globals():
	ARM = 's3a'

import subprocess
def run_for_stdout(str_cmd):
	r = subprocess.run(str_cmd.split(), stdout = subprocess.PIPE)
	return r.stdout.decode('utf-8')
on_rpi = run_for_stdout('lsb_release -i') == 'Distributor ID:	Raspbian\n'

arm_main = f'{ARM}_main'
if not on_rpi:
	if ARM == 's3a':
		arm_main = f'{ARM}_serial_main'

_settings = {
	'SIM': SIM,
	'ARM': ARM,
	'arm_main': arm_main,
}



_layout = [
	{
		'drv':
		# Tuple is vert pane split, while list is horiz.
		(
			['driver', 'app'],
			'dmesg',
		)
	},
	# Key is win name, value are panes.
	{
		'ros1':
		# Tuple is vert pane split, while list is horiz.
		(
			'main',
			['teleop', 'routine'],
		)
	},
	{
		'ros2':
		(
			['joint_echo', 'pose_echo'],
			['joy_echo', 'cmd'],
		)
	},
	{ 'joints': 'joints' },
	{ 'playground': 'playground' },
]

build_drv = {
	'driver' : '''
		make
		''',
	'app' : '''
		./waf configure
		./waf build
		''',
}
# Key is target name, value is list of batches.
_targets = {
	# Done before everything.
	'setup': [
		{
			'driver': 'cd ../../SW/Driver/motor_ctrl/',
			'app': 'cd ../../SW/Test/test_app/',
			'dmesg': 'cd ../../SW/Driver/motor_ctrl/',
		}
		
	],
	'build_drv' : [
		build_drv
	],
	'build': [
		# Batch where key is pane where to execute,
		# value is multiline str of cmds.
		{
			**build_drv,
			'main': '''
				catkin_make -j2
				''',
		},
	],
	'__common_run_drv': [
		{
			'driver' : '''
				make stop
				make start
				''',
			'dmesg' : '''
				tmux_no_wait
				dmesg -w
				''',
		},
	],
	'run_drv': [
		{
			'app' : '''
				./build/test_servos w 0 500 NO_ENTER
				''',
		},
	],
	'__run_drv_wc': [
		{
			'app' : '''
				./build/test_bldc 0
				''',
		},
	],
	'__common_run': [
		{
			pane : 'source devel/setup.bash' for pane in [
				'main',
				'routine',
				'teleop',
				'joint_echo',
				'pose_echo',
				'joy_echo',
				'cmd',
				'joints',
			]
		},
	],
	'run': [
		{
			# Мultiline str: every line new cmd.
			# Backslash (\) before new line means continuation of line
			# NO_ENTER means that command is just pasted, not execute,
			# i.e. no ENTER will be send to tmux pane.
			'main': '''
				tmux_no_wait
				roslaunch ${arm_main} main.launch \
					all_motors_sim:=${SIM} \
					small_screen:=true
				''',
			'teleop': '''
				sleep 30
				''',
			'playground': '''
				function list_controllers(){ \
					rosrun controller_manager controller_manager list; \
				}
				function traj(){ \
					rosrun common_teleop change_controller.py traj; \
				}
				tmux_exit NO_ENTER
				''',
		},
		{
			'routine': '''
				roslaunch common_teleop routines_teleop.launch name:=${ARM}
				''',
			'teleop': '''
				# Joypad must have analog on.
				roslaunch arm_teleop jog_teleop.launch name:=${ARM}
				#roslaunch arm_teleop servo_teleop.launch name:=${ARM}
				''',
			'joint_echo': '''
				rostopic echo /joint_states NO_ENTER
				''',
			'pose_echo': '''
				rostopic echo /tf_publish/beam00_base__to__beam2_hand_ee NO_ENTER
				''',
			'joy_echo': '''
				rostopic echo /servo_server/delta_twist_cmds NO_ENTER
				#rostopic echo /vision_teleop/delta_twist_cmds
				''',
			'cmd': '''
				rosrun moveit_commander moveit_commander_cmdline.py arm NO_ENTER
				''',
			'joints': '''
				tail --follow \
					`roslaunch-logs`/robot_hardware_interface__joints.log
				''',
		},
	],
	'run_wc': [
		{
			'main': '''
				tmux_no_wait
				roslaunch wc_main main.launch \
					all_motors_sim:=${SIM} \
					small_screen:=true
				''',
			'teleop': '''
				sleep 20
				''',
			'playground': '''
				function list_controllers(){ \
					rosrun controller_manager controller_manager list; \
				}
				function traj(){ \
					rosrun common_teleop change_controller.py traj; \
				}
				tmux_exit NO_ENTER
				''',
		},
		{
			'routine': '''
				tmux_no_wait
				roslaunch common_teleop routines_teleop.launch name:=${ARM}
				''',
			'teleop': '''
				tmux_no_wait
				# Joypad must have analog on.
				roslaunch wc_teleop manual_teleop.launch
				''',
		},
	]

}

# Do not wait in last batch.
last_batch = _targets['run'][-1]
no_wait_last_batch = {}
for pane, cmds in last_batch.items():
	cmds = 'tmux_no_wait\n' + cmds
	no_wait_last_batch[pane] = cmds
_targets['run'][-1] = no_wait_last_batch

_dependencies = {
	'build_drv': ['setup'],
	'build': ['build_drv'],
	'run_drv': ['setup', '__common_run_drv'],
	'__run_drv_wc': ['setup', '__common_run_drv'],
	'run': ['run_drv', '__common_run'],
	'run_wc': ['__run_drv_wc', '__common_run'],
}


