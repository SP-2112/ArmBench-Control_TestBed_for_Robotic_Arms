import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yash_sai/Yash/Arm/InterbotixDriver/install/interbotix_common_modules'
