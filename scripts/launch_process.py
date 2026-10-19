"""Background shell jobs inherit ignored SIGINT; restore it before exec launch."""
import os
import signal
import sys

signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execvp('ros2', ['ros2', 'launch', 'lbr_pybullet_tutorial', 'tutorial.launch.py', *sys.argv[1:]])
