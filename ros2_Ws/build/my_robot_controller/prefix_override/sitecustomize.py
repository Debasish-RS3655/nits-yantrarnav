import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/f/NITS/ANAV/main/ros2_Ws/install/my_robot_controller'
