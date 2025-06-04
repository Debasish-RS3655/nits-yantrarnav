#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    # List of (executable, human-readable description) tuples:
    nodes = [
        ("bridge_server",       "[1] bridge_server"),
        ("launch_checker",      "[2] launch_checker"),
        ("path_planner",        "[3] path_planner"),
        ("height_publisher",    "[4] height_publisher"),
        ("flat_area",           "[5] flat_area"),
        ("boundary_mapper",     "[6] boundary_mapper"),
        ("coordinate",          "[7] coordinate"),
    ]

    for exe, desc in nodes:
        # Each node runs in its own gnome-terminal on DISPLAY=:0
        ld.add_action(
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    # Make sure ROS 2 and your workspace are sourced, then run the node.
                    # 'exec bash' keeps the terminal open after the node exits.
                    "export DISPLAY=:0 && "
                    # "source /opt/ros/humble/setup.bash && "
                    # "source ~/ros2_ws/install/setup.bash && "
                    f"ros2 run my_robot_controller {exe}; exec bash"
                ],
                output="screen"
            )
        )
        # (Optional) You can insert a small delay between terminals if needed:
        # ld.add_action(TimerAction(period=0.5, actions=[]))

    return ld
