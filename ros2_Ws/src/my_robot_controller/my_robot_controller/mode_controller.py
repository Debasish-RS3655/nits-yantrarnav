#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading
import time

# manual mode nodes
from .path_manual import PathManual
# auto mode nodes
from .variations.path_planner_running_latest import PathPlanner

# Debashish Buragohain
# this code subscribes to the mode topic and launches the automatic or manual nodes depending on that

class ModeExecutor(Node):
    def __init__(self):
        super().__init__('mode_executor')
        self.system_status_sub = self.create_subscription(String, 'system_launch_status', self.update_launch, 10)
        self.system_ready = False
        
        self.publisher_ = self.create_subscription(String, 'position/mode', self.update_mode,  10)
        # Set initial mode here, and you can update it based on user input or service calls.
        self.mode = "manual"

        # list to hold currently ruunning nodes and their threads
        # each entry will be a tuple: (node_instance, thread, shutdown_event)
        self.running_nodes = []
        self.get_logger().info(f"Initialized ModeExecutor in {self.mode} mode.")

    def update_launch(self, msg:Bool):
        if msg.data == True:
            if not self.system_ready == True:
                self.system_ready = True
                self.get_logger().info("System state is ready in mode controller.")                

    # update mode if a new mode is provided
    def update_mode(self, msg: String):        
        new_mode = msg.data        
        # return if the system is not ready
        if self.system_ready == False:
            return
        
        self.get_logger().info("Received mode update: " + str(self.mode))        
        if new_mode not in ['automatic', 'manual']:
            self.get_logger().error("Invalid mode received: " + new_mode)
            return

        if new_mode == self.mode:
            self.get_logger().info("Mode unchanged. No action taken.")
            return
        
        # if the logic reaches here means the mode has been changed
        # Mode changed: shutdown current group and start new group.
        self.shutdown_running_nodes()
        self.start_nodes_for_mode(new_mode)
        self.mode = new_mode
    
    def shutdown_running_nodes(self):
        self.get_logger().info("Shutting down current mode nodes...")
        for node_inst, thread, shutdown_evt in self.running_nodes:
            shutdown_evt.set()  # signal shutdown for this node
            if thread is not threading.current_thread():
                thread.join(timeout=5)
            node_inst.destroy_node()
        self.running_nodes.clear()
        self.get_logger().info("All nodes shut down.")

        
    def start_nodes_for_mode(self, mode):
        self.get_logger().info(f"Start")
        # define the node classes for each node
        if mode == "automatic":
            node_classes = [PathPlanner]
        elif mode == "manual":
            node_classes = [PathManual]
        else:
            self.get_logger().error(f"Unsupported mode: {mode}")
            return
    
        for NodeClass in node_classes:
            node_inst = NodeClass() # instantiate that node
            shutdown_evt = threading.Event()
            thread = threading.Thread(target=self.spin_node, args=(node_inst, shutdown_evt))
            thread.start()
            self.running_nodes.append((node_inst, thread, shutdown_evt))
            self.get_logger().info(f"Started node: {node_inst.get_name()}")
            
    def spin_node(self, node, shutdown_evt):
        """Helper function to spin a ROS node until shutdown_evt is set."""
        try:
            while not shutdown_evt.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    mode_executor = ModeExecutor()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(mode_executor)
    try:
        executor.spin()
    except KeyboardInterrupt:
        mode_executor.get_logger().info("KeyboardInterrupt received. Shutting down ModeExecutor.")
    finally:
        executor.shutdown()
        mode_executor.shutdown_running_nodes()
        mode_executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()