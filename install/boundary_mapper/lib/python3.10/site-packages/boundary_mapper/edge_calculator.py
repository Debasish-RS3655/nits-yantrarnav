#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as GeoPoint
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
        # publisher for markers
        self.marker_pub = self.create_publisher(MarkerArray, 'edge_markers', 10)
        
        self.get_logger().info("EdgeCalculator node has started.")

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
                # publish visualization markers
                self.publish_markers(coords)
                # reset list to wait for next pair
                self.edge_list.clear()

    def publish_markers(self, coords):
        ma = MarkerArray()
        # marker for each corner
        for idx, p in enumerate(coords):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'edge_corners'
            m.id = idx
            m.type = Marker.SPHERE
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0
            m.pose.position = GeoPoint(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            ma.markers.append(m)
        # optional: connect corners with line
        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'edge_outline'
        line.id = len(coords)
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.05
        line.color.r = 1.0; line.color.g = 1.0; line.color.b = 0.0; line.color.a = 1.0
                
        for p in coords:
            line.points.append(GeoPoint(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        # close rectangle
        line.points.append(GeoPoint(x=float(coords[0][0]), y=float(coords[0][1]), z=float(coords[0][2])))
        ma.markers.append(line)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = EdgeCalculator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
