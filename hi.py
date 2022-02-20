import sys
import importlib

args = sys.argv
quad_lib = importlib.import_module(sys.argv[1])
q = quad_lib.quadruped()