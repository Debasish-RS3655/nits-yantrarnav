from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    mapper = Node(
        package="boundary_mapper",
        executable="mapper"
    )    
    
    navigator = Node(
        package="boundary_mapper",
        executable="navigator"
    )    
    
    edge_calculator = Node(
        package="boundary_mapper",
        executable="edge_calculator"
    )    
        
    ld.add_action(mapper) 
    ld.add_action(navigator) 
    ld.add_action(edge_calculator)         
           
    return ld

