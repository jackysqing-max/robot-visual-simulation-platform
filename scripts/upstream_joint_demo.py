#!/usr/bin/env python3
"""Execute the unmodified, pinned LBR Stack Python trajectory client in simulation."""
import importlib.util
from pathlib import Path
import sys

source = Path(__file__).resolve().parents[1]/'third_party/lbr_stack/joint_trajectory_client.py'
spec = importlib.util.spec_from_file_location('lbr_stack_upstream_demo', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.main(args=sys.argv[1:])
