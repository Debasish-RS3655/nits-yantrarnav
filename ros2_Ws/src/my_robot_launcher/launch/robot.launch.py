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
    
    path_mover = Node(
        package="my_robot_controller",
        executable="path_mover"
    )        

    system_launch_checker = Node(
        package="my_robot_controller",
        executable="launch_checker"
    )

    # dummy nodes for testing
    flat_area = Node(
        package="my_robot_controller",
        executable="flat_area_dummy"
    )
    
    boundary_mapper = Node(
        package="my_robot_controller",
        executable="boundary_mapper_dummy"
    )
    
    odom = Node(
        package="my_robot_controller",
        executable="odom_dummy"        
    )
    
    path_planner = Node(
        package="my_robot_controller",
        executable="path_planner"        
    )
    
    ld.add_action(bridge_server)
    ld.add_action(path_mover)
    ld.add_action(system_launch_checker)
    ld.add_action(path_planner)

    ld.add_action(flat_area)    
    ld.add_action(boundary_mapper)
    ld.add_action(odom)
    
        
    return ld

