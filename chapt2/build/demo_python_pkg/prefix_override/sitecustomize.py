import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/edward/ros2_demo/chapt2/install/demo_python_pkg'
