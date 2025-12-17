import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yash_sai/Yash/Arm/ros_ws/install/config_teleop'
