from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    # add nodes
    firstnode = Node(
        package="my_robot_controller",
        executable="bridge_server"
    )
    
    ld.add_action(firstnode)
    return ld

