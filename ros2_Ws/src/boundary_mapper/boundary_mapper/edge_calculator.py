#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

class EdgeCalculator(Node):
    def __init__(self):
        super().__init__('edge_calculator_py')
        self.declare_parameter('tol', 0.5)
        self.tol = self.get_parameter('tol').value

        # Subscribe to boundary_edges String topic
        self.edge_list = []  # store detected edges as tuples (A, C)
        self.create_subscription(String, 'boundary_edges', self.cb, 10)
        self.pub = self.create_publisher(String, 'fence_rectangle', 10)

    def cb(self, msg: String):
        data = msg.data
        if not data:
            return
        parts = data.split(':', 1)
        if len(parts) != 2:
            return
        typ, coords_str = parts
        if typ != 'edge':
            return
        pts = []
        for pt_str in coords_str.split(';'):
            vals = [v for v in pt_str.strip().split(',') if v]
            if len(vals) != 3:
                continue
            x, y, z = map(float, vals)
            pts.append(np.array([x, y, z]))
        if len(pts) != 3:
            return
        # edge provides three points: A, I, C; we use A and C as endpoints
        A, _, C = pts
        # store this edge
        self.edge_list.append((A, C))
        # keep only last two edges
        if len(self.edge_list) > 2:
            self.edge_list.pop(0)
        # check if we have two distinct edges
        if len(self.edge_list) == 2:
            A1, C1 = self.edge_list[0]
            A2, C2 = self.edge_list[1]
            # check distance between midpoints > 5 m
            mid1 = (A1 + C1) / 2.0
            mid2 = (A2 + C2) / 2.0
            if np.linalg.norm(mid2 - mid1) >= 5.0:
                # publish four edge coordinates: A1,C1,A2,C2
                coords = [A1, C1, A2, C2]
                pairs = [f"{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}" for p in coords]
                out_msg = String()
                out_msg.data = ",".join(pairs)
                self.pub.publish(out_msg)
                self.get_logger().info(f"Published rectangle corners: {out_msg.data}")
                # reset list to wait for next pair
                self.edge_list.clear()


def main():
    rclpy.init()
    node = EdgeCalculator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
