#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Makefile with tmux
'''

###############################################################################

__author__    = 'Milos Subotic'
__email__     = 'milos.subotic.sm@gmail.com'
__copyright__ = 'MIT'

###############################################################################

import os
import sys

###############################################################################

import re
import inspect

# Whole expression.
_show_regex = re.compile(r'\bshow\s*\(\s*(.*)\s*\)')

def show(var):
	varName = ''
	for line in inspect.getframeinfo(inspect.currentframe().f_back)[3]:
		m = _show_regex.search(line)
		if m:
			varName = m.group(1)
			break
	print('{0} = {1}'.format(varName, var))


###############################################################################

import sys

VERB  = 0
DEBUG = 1
INFO  = 2
WARN  = 3
ERROR = 4
FATAL = 5

__MSG_PRINT_TYPE = True
def msg_print_type(tf):
	global __MSG_PRINT_TYPE
	__MSG_PRINT_TYPE = tf

def msg(msg_type, *args, **kwargs):
	global __MSG_PRINT_TYPE
	if msg_type == VERB:
		color = "\x1b[37m"
		msg_type_str = "verbose"
	elif msg_type == DEBUG:
		color = "\x1b[92m"
		msg_type_str = "debug"
	elif msg_type == INFO:
		color = "\x1b[94m"
		msg_type_str = "info"
	elif msg_type == WARN:
		color = "\x1b[93m"
		msg_type_str = "warning"
	elif msg_type == ERROR:
		color = "\x1b[91m"
		msg_type_str = "error"
	elif msg_type == FATAL:
		color = "\x1b[91m"
		msg_type_str = "fatal"
	else:
		raise AssertError("Wrong msg_type!")
	
	if __MSG_PRINT_TYPE:
		m = msg_type_str + ":"
	else:
		m = ""
	
	print(color + m, sep = '', end = '')
	print(*args, **kwargs, sep = '', end = '')
	print("\x1b[0m", sep = '', end = '') # Return to normal.
	print()

	if msg_type == FATAL:
		sys.exit(1)


def warn(*args, **kwargs):
	print('WARN: ', *args, file = sys.stderr, **kwargs)

def error(*args, **kwargs):
	print('ERROR: ', *args, file = sys.stderr, **kwargs)
	#sys.exit(1)
	raise RuntimeError(1)

###############################################################################

class Parser:
	COMMAND = 0
	COMMENT = 1
	BACKSLASH = 2
	SINGLE_QUOTE = 3
	DOUBLE_QUOTE = 4
		
	def eat_word(self):
		if self.word != '':
			self.lst_cmd.append(self.word)
		self.word = ''
	def eat_lst_cmd(self):
		if self.lst_cmd != []:
			self.lst_cmds.append(self.lst_cmd)
		self.lst_cmd = []
		
	def __init__(self, str_cmds):
		if not isinstance(str_cmds, str):
			error('Not supported cmds: ', str_cmds)
		else:
			self.lst_cmds = [] # List of lists.
			self.lst_cmd = []
			self.word = ''
			
			state = Parser.COMMAND
			for c in str_cmds:
				if state == Parser.COMMAND:
					if c in [' ', '\t', '\n', '#']:
						self.eat_word()
						if c in ['\n', '#']:
							self.eat_lst_cmd()
							if c == '#':
								state = Parser.COMMENT
					elif c == '\\':
						state = Parser.BACKSLASH
					elif c == "'":
						state = Parser.SINGLE_QUOTE
					elif c == '"':
						state = Parser.DOUBLE_QUOTE
					else:
						self.word += c
				elif state == Parser.COMMENT:
					if c == '\n':
						state = Parser.COMMAND
				elif state == Parser.BACKSLASH:
					if c == '\n':
						# Continuation of line.
						eat_word()
					else:
						self.word += '\\'
						self.word += c
					state = Parser.IDLE
				elif state == Parser.SINGLE_QUOTE:
					if c == "'":
						state = Parser.COMMAND
					else:
						self.word += c
				elif state == Parser.DOUBLE_QUOTE:
					if c == '"':
						state = Parser.COMMAND
					else:
						self.word += c
					

			self.eat_word()
			self.eat_lst_cmd()
			

def parse_cmds(str_cmds):
	return Parser(str_cmds).lst_cmds

import subprocess

def _run_for_exit_code(lst_cmd):
	assert(isinstance(lst_cmd, list))
	r = subprocess.run(lst_cmd)
	return r.returncode

def join_lst_cmd(lst_cmd):
	joined_str_cmd = ''
	for word in lst_cmd:
		have_white = False
		have_double_quote = False
		for c in word:
			if c in [' ', '\t']:
				have_white = True
			if c == '"':
				have_double_quote = True
		if have_white:
			joined_str_cmd += '"'
			if have_double_quote:
				word = word.replace('"', '\\"')
			joined_str_cmd += word
			joined_str_cmd += '"'
		else:
			joined_str_cmd += word
		joined_str_cmd += ' '
		
	joined_str_cmd = joined_str_cmd[0:-1] # Removing last space.
	return joined_str_cmd

def run_cmd(lst_cmd):
	ok = _run_for_exit_code(lst_cmd) == 0
	if not ok:
		error('failed cmd:', join_lst_cmd(lst_cmd))

def run(cmds):
	lst_cmds = parse_cmds(cmds)
	for lst_cmd in lst_cmds:
		print(join_lst_cmd(lst_cmd))
		run_cmd(lst_cmd)

#def run_for_stdout(cmd):
#	assert(isinstance(cmd, list))
#	r = subprocess.run(cmd, stdout = subprocess.PIPE)
#	return r.stdout.decode('utf-8')
	
	
###############################################################################

import argparse

parser = argparse.ArgumentParser(
	prog = 'tmuxer.py',
	description = 'setup tmux consoles and exec cmds in them'
)
parser.add_argument(
	'--tmux-reserved',
	action='store_true',
	help = 'Reserved'
)
parser.add_argument(
	'targets_and_settings',
	metavar = 'targets_and_settings',
	nargs = '*',
	help = 'targets (like build, run) or settings (like SIM=true)'
)
args = parser.parse_args()

################

targets = []
settings = []
for target_or_setting in args.targets_and_settings:
	if '=' in target_or_setting:
		key_value = target_or_setting.split("=")
		settings.append(key_value)
	else:
		targets.append(target_or_setting)
	
################

# Install dependencies.
import shutil
for_install = []
for p in ['tmux', 'xsel']:
	if not shutil.which(p):
		for_install.append(p)
if len(for_install):
	print('Need to install packages')
	lst_cmd = f'sudo apt -y install'.split() + for_install
	print(join_lst_cmd(lst_cmd))
	run_cmd(lst_cmd)


################


cfg_fn = os.path.join(os.getcwd(), 'tmuxer.cfg.py')
# Check if config file exists.
if not os.path.exists(cfg_fn):
	error(f'No config file "{cfg_fn}"')
cfg_code = open(cfg_fn, 'r').read()

def tryparse(type, value):
	try:
		return type(value)
	except ValueError:
		return None
def conv_for_python(s):
	i = tryparse(int, value)
	if i != None:
		return i
	sl = s.lower()
	t = sl in ['true', '1', 'yes', 'y']
	f = sl in ['false', '0', 'no', 'n']
	if t:
		return True
	if f:
		return False
	#TODO Better escaping
	return "'" + s + "'"
	

settings_code = ''
for key, value in settings:
	v = conv_for_python(value)
	settings_code += f'{key} = {v}\n'
	

class Cfg:
	def __init__(self):
		self._layout = []
		self._settings = {}
		self._targets = {}
		self._dependencies = {}
cfg = Cfg()
cfg_obj = compile(settings_code+cfg_code, '', 'exec')
exec(cfg_obj, cfg.__dict__)

################


if not args.tmux_reserved:
	lst_cmd = []
	lst_cmd.append(sys.argv[0])
	lst_cmd.append('--tmux-reserved')
	lst_cmd += sys.argv[1:]
	
	################
	# Settings.
	
	env = os.environ.copy()
	
	for key, value in cfg._settings.items():
		env[key] = str(value)
	
	env['force_color_prompt'] = 'yes' # To have colors.
	for setting in settings:
		key, value = setting
		env[key] = value
	
	################
	
	tmux_lst_cmd = ['tmux', 'new', join_lst_cmd(lst_cmd)]
	r = subprocess.run(tmux_lst_cmd, env = env)
	sys.exit(r.returncode)
	# Not executing anything below this line.

try:

	################
	# Setup.

	run('''
		tmux rename-window "_tmux"
		tmux select-pane -t _tmux.0 -T _tmux
		# Scroll on mouse wheel. Same action also enters copy mode.
		tmux set -g mouse on
		# Copy to clipboard on Ctrl+C.
		tmux bind -T copy-mode C-c send -X copy-pipe-no-clear "xsel -i --clipboard"
		# Drag select.
		tmux unbind -T copy-mode MouseDragEnd1Pane
		tmux set -g pane-border-status top
		#tmux set -g pane-border-format "#{pane_index} #{pane_current_command}"
	''')

	################
	# Make panes

	wins = ['_tmux']
	pane_to_id = {'_tmux' : 'tmux.0'}
	for win_tree in cfg._layout:
		for win, pane_tree in win_tree.items():
			if win in wins:
				error('Have duplicate win name "{}"'.format(win))
			else:
				run('tmux new-window -n "{}"'.format(win))
				def split(pane_or_tree, idx):
					if isinstance(pane_or_tree, str):
						# Leaf
						pane = pane_or_tree
						if pane in pane_to_id:
							error('Have duplicate pane name "{}"'.format(pane))
						id = win + '.{}'.format(idx)
						idx += 1
						pane_to_id[pane] = id
						run('tmux select-pane -t {} -T {}'.format(id, pane))
					else:
						tree = pane_or_tree
						if isinstance(tree, tuple):
							split_way = 'v'
						elif isinstance(tree, list):
							split_way = 'h'
						else:
							error(
								'Pane tree cannot have type '
									+ '{}'.format(type(tree))
							)
						id = win + '.{}'.format(idx)
						run('tmux select-pane -t {}'.format(id))
						
						N_splits = len(tree)-1
						for i in range(N_splits):
							run('tmux split-window -{}'.format(split_way))
						
						for sub_tree in tree:
							idx = split(sub_tree, idx)

						
					return idx # Next free.
					
				split(pane_tree, 0)

	################
	# Wait to got prompts.
	
	import time
	time.sleep(1.0)

	################
	
	def send_cmd(pane, lst_cmd):
		send_lst_cmd = 'tmux send -t'.split()
		send_lst_cmd.append(pane_to_id[pane])
		no_enter = lst_cmd[-1] == 'NO_ENTER'
		if no_enter:
			# Just paste cmd.
			send_lst_cmd.append(join_lst_cmd(lst_cmd[0:-1]))
		else:
			send_lst_cmd.append(join_lst_cmd(lst_cmd))
			send_lst_cmd.append('ENTER')
			
		print(pane, '<<<', join_lst_cmd(lst_cmd))
		run_cmd(send_lst_cmd)
		
		have_sudo = 'sudo' in lst_cmd
		if have_sudo:
			#TODO tab:pane
			sys.stdin.flush()
			input(
				f'Pane {pane} executing sudo. '\
					'Press enter to when type into the password, if any...'
			)
		
		need_to_wait = not no_enter
		return need_to_wait
	
	################
	
	# To all panes.
	for pane in pane_to_id.keys():
		if pane == "_tmux":
			continue
		
		def send_str_cmd(str_cmd):
			lst_cmd = str_cmd.split()
			send_cmd(pane, lst_cmd)
			
		send_str_cmd(
			f'function tmux_no_wait(){{ return 0; }}'
		)
		send_str_cmd(
			f'function tmux_done_batch(){{ tmux wait -S "__done__{pane}"; }}'
		)
		send_str_cmd('''
			function tmux_done_batch_for(){ \
				pane=$1; \
				tmux wait -S "__done__${pane}"; \
			}
		''')
		send_str_cmd('function tmux_exit(){ tmux kill-session; }')
		send_str_cmd('clear')
		
		
		
	################
	# Exec targets.
		
	def exec_cmds(batch_name, pane, cmds):
		if isinstance(cmds, str):
			lst_cmds = parse_cmds(cmds)
			wait_enable = True
			need_to_wait = False
			for lst_cmd in lst_cmds:
				if lst_cmd == ['tmux_no_wait']:
					wait_enable = False
				need_to_wait = send_cmd(pane, lst_cmd)
				
			if wait_enable:
				if need_to_wait:
					# Last not just paste of cmd. Indicate it is done.
					lst_cmd = ['tmux_done_batch']
					lst_cmd.append(batch_name)
					send_cmd(pane, lst_cmd)
				return need_to_wait
			else:
				return False
		elif cmds == False:
			# Shortcut for disabling some commands.
			return False
		else:
			error(
				'Wrong type "{}" in cmd tree {}!'.format(
					type(cmds),
					cmds
				)
			)
	
	def exec_batch(batch_name, batch):
		if isinstance(batch, dict):
			panes_to_wait = []
			for pane, cmds in batch.items():
				need_to_wait = exec_cmds(batch_name, pane, cmds)
				if need_to_wait:
					panes_to_wait.append(pane)
				
			print(f'Waiting for {panes_to_wait}')
			
			# Do not go to the next batch if this is not done.
			for i, pane in enumerate(panes_to_wait):
				lst_cmd = 'tmux wait'.split()
				lst_cmd.append('__done__' + pane)
				run_cmd(lst_cmd)
				print(f'Pane "{pane}" done. Waiting for {panes_to_wait[i+1:]}')
			print(f'Done batch {batch_name}')
		else:
			error(
				'Wrong type "{}" in cmd tree {}!'.format(
					type(batch),
					batch
				)
			)
	
	def exec_target(target, list_of_batches):
		if isinstance(list_of_batches, list):
			N_batches = len(list_of_batches)
			for batch_idx, batch in enumerate(list_of_batches, start = 1):
				batch_name = f'{target}:{batch_idx}/{N_batches}'
				exec_batch(batch_name, batch)
		else:
			error(
				'Wrong type "{}" in cmd tree {}!'.format(
					type(list_of_batches),
					list_of_batches
				)
			)
	
	done_targets = []
	for target in targets:
		if target in cfg._targets:
			def exec_with_dependencies(t):
				if not t in done_targets:
					if t in cfg._dependencies:
						for d in cfg._dependencies[t]:
							exec_with_dependencies(d)
					exec_target(t, cfg._targets[t])
					done_targets.append(t)
			exec_with_dependencies(target)
		else:
			error(
				'target "{}" does not exists'\
					' in list of targets in tmuxer.cfg.py'.format(target)
			)
	raise KeyboardInterrupt(1)
	

except Exception as err:
	print('An exception occurred: ', type(err).__name__, ': ', err)
	import traceback
	print(traceback.format_exc())
except KeyboardInterrupt as err:
	print('KeyboardInterrupt')
except RuntimeError as err:
	print('RuntimeError')
except:
	print('Unexpected')
	
###############################################################################

# Last thing.
sys.stdin.flush()
input('Press enter to exit tmux session...')
run('tmux kill-session')

###############################################################################
