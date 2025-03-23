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

    system_launch_checker = Node(
        package="my_robot_controller",
        executable="launch_checker"
    )

    # in the latest version we launch the mode controller and the path planner and manual mode are set from there
    path_planner = Node(
        package="my_robot_controller",
        executable="mode_controller"
    )
    
    # # demo packages
    # demo_launch_node1 = Node(
    #     package="my_robot_controller",
    #     executable="delayed_launch_node1"
    # )
    
    # demo_launch_node2 = Node(
    #     package="my_robot_controller",
    #     executable="delayed_launch_node2"
    # )    
    
    ld.add_action(bridge_server)
    ld.add_action(flat_area)
    ld.add_action(path_mover)
    ld.add_action(system_launch_checker)
    ld.add_action(path_planner)
    
    # ld.add_action(demo_launch_node1)
    # ld.add_action(demo_launch_node2)
        
    return ld

