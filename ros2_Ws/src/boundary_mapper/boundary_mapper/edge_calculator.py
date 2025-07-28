#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from boundary_mapper.msg import FenceCorners
from std_msgs.msg import String
import numpy as np

class EdgeCalculator(Node):
    def __init__(self):
        super().__init__('edge_calculator')
        self.sub = self.create_subscription(
            FenceCorners, 'fence_corners', self.cb, 10)
        self.pub = self.create_publisher(String, 'fence_rectangle', 10)
        self.declare_parameter('tol', 0.5)  # tolerance in meters

    def cb(self, msg: FenceCorners):
        if msg.status != 'ok' or len(msg.corners) < 2:
            return
        
        p1 = msg.corners[0]
        p2 = msg.corners[1]
        dir3 = msg.direction
        tol = self.get_parameter('tol').value

        # convert to numpy for math
        P1 = np.array([p1.x, p1.y, p1.z])
        P2 = np.array([p2.x, p2.y, p2.z])
        d  = np.array([dir3.x, dir3.y, dir3.z])
        # perpendicular in XY-plane
        perp = np.array([-d[1], d[0], 0.0])

        dist = np.linalg.norm(P2 - P1)
        # decide which side is along this pair
        if abs(dist - 9.0) < tol:
            short, long = 9.0, 12.0
        elif abs(dist - 12.0) < tol:
            short, long = 12.0, 9.0
        else:
            self.get_logger().warn(f"Unexpected edge distance: {dist:.2f} m")
            return

        # The given points are one pair of opposite edges separated by 'short'.
        # We now offset them along 'perp' by 'long' to get the other two corners.
        P3 = P2 + perp * long
        P4 = P1 + perp * long

        # format as a single string
        coords = [
            ("x1", P1[0], "y1", P1[1], "z1", P1[2]),
            ("x2", P2[0], "y2", P2[1], "z2", P2[2]),
            ("x3", P3[0], "y3", P3[1], "z3", P3[2]),
            ("x4", P4[0], "y4", P4[1], "z4", P4[2]),
        ]
        s = ", ".join(f"{n}={v:.2f}" for tup in coords for n,v in zip(tup[::2], tup[1::2]))
        msg_out = String()
        msg_out.data = s
        self.pub.publish(msg_out)
        self.get_logger().info(f"Published rectangle corners: {s}")

def main():
    rclpy.init()
    node = EdgeCalculator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
