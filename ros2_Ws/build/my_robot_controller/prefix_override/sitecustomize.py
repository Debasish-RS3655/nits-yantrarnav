import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nits/master_controller/nits-yantrarnav/ros2_Ws/install/my_robot_controller'
