from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    # add nodes
    bridge_server = Node(
        package="my_robot_controller",
        executable="bridge_server",
        sigterm_timeout='2',  # give the node 2 seconds to shut down gracefully
        sigkill_timeout='5'   # force kill if it does not exit within 5 seconds
    )
    
    flat_area = Node(
        package="my_robot_controller",
        executable="flat_area"
    )
    
    path_mover = Node(
        package="my_robot_controller",
        executable="path_mover"
    )        

    path_planner = Node(
        package="my_robot_controller",
        executable="path_planner"
    )
    
    ld.add_action(bridge_server)
    ld.add_action(flat_area)
    ld.add_action(path_mover)
    ld.add_action(path_planner)
        
    return ld

