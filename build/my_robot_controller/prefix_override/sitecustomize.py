import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nits/master_controller/nits-yantrarnav/install/my_robot_controller'
