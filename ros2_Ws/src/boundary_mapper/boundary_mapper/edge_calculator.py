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
        # Params
        self.declare_parameter('tol', 0.5)                        # (kept for compatibility; unused)
        self.declare_parameter('min_edge_separation', 7.5)        # meters between saved corners (I points)
        self.declare_parameter('marker_frame', 'map')
        self.declare_parameter('corner_marker_scale', 0.25)       # meters (sphere diameter)
        self.declare_parameter('line_width', 0.07)                # meters

        self.tol = self.get_parameter('tol').value
        self.min_edge_separation = self.get_parameter('min_edge_separation').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.corner_marker_scale = float(self.get_parameter('corner_marker_scale').value)
        self.line_width = float(self.get_parameter('line_width').value)

        # State
        self.saved_edges = []      # list of dicts: {'A':np3, 'I':np3, 'C':np3}
        self.initial_corner_I = None
        self.corner_count = 0      # for marker IDs

        # I/O
        self.create_subscription(String, 'boundary_edges', self.cb, 10)
        self.pub_rect = self.create_publisher(String, 'fence_rectangle', 10)
        self.pub_saved_corner = self.create_publisher(String, 'saved_corner', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'edge_markers', 10)

        self.get_logger().info("EdgeCalculator node has started.")

    def parse_edge(self, msg_data: str):
        """
        Expects 'edge:x1,y1,z1;x2,y2,z2;x3,y3,z3'
        Returns (A, I, C) as np arrays or None.
        """
        parts = msg_data.split(':', 1)
        if len(parts) != 2:
            return None
        typ, coords_str = parts
        if typ != 'edge':
            return None
        pts = []
        for pt_str in coords_str.split(';'):
            vals = [v for v in pt_str.strip().split(',') if v]
            if len(vals) != 3:
                continue
            try:
                x, y, z = map(float, vals)
            except ValueError:
                return None
            pts.append(np.array([x, y, z], dtype=float))
        if len(pts) != 3:
            return None
        A, I, C = pts
        return A, I, C

    def cb(self, msg: String):
        parsed = self.parse_edge(msg.data)
        if not parsed:
            return

        A, I, C = parsed

        # Gate by separation from the first saved corner
        if self.initial_corner_I is None:
            # Accept first corner
            self.initial_corner_I = I.copy()
            self.saved_edges = [ {'A': A, 'I': I, 'C': C} ]
            self.on_corner_saved(I)
            self.visualize_corners([I])
            return
        else:
            dist = np.linalg.norm(I - self.initial_corner_I)
            if dist < self.min_edge_separation:
                self.get_logger().info(
                    f"Ignoring edge: corner separation {dist:.2f} m "
                    f"< {self.min_edge_separation:.2f} m from initial corner.")
                return
            # Accept as second edge
            self.saved_edges.append({'A': A, 'I': I, 'C': C})
            self.on_corner_saved(I)
            self.visualize_corners([I])

        # If we now have two saved edges, publish rectangle and visualize
        if len(self.saved_edges) == 2:
            A1, C1 = self.saved_edges[0]['A'], self.saved_edges[0]['C']
            A2, C2 = self.saved_edges[1]['A'], self.saved_edges[1]['C']
            coords = [A1, C1, A2, C2]
            pairs = [f"{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}" for p in coords]

            out = String()
            out.data = ",".join(pairs)
            self.pub_rect.publish(out)
            self.get_logger().info(f"Published rectangle corners: {out.data}")

            self.visualize_rectangle(coords)

            # Reset to seek a new pair next time
            self.saved_edges.clear()
            self.initial_corner_I = None

    # ---------- Publishing helpers ----------

    def on_corner_saved(self, I):
        """Publish the saved corner coordinate and log."""
        msg = String()
        msg.data = f"{I[0]:.3f},{I[1]:.3f},{I[2]:.3f}"
        self.pub_saved_corner.publish(msg)
        self.get_logger().info(f"Saved corner: {msg.data}")

    def visualize_corners(self, corners_np_list):
        """Add corner spheres for each saved corner received."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for I in corners_np_list:
            m = Marker()
            m.header.frame_id = self.marker_frame
            m.header.stamp = now
            m.ns = 'saved_corners'
            m.id = self.corner_count
            self.corner_count += 1
            m.type = Marker.SPHERE
            m.scale.x = m.scale.y = m.scale.z = self.corner_marker_scale
            m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 1.0
            m.pose.position = GeoPoint(x=float(I[0]), y=float(I[1]), z=float(I[2]))
            ma.markers.append(m)

        self.marker_pub.publish(ma)

    def visualize_rectangle(self, coords):
        """
        coords = [A1, C1, A2, C2]; draw spheres at corners and a polyline around them.
        The order here just draws A1->C1->A2->C2->A1 (not necessarily the true rectangle order,
        but enough to visualize the two detected edges as a loop).
        """
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Corner markers
        for idx, p in enumerate(coords):
            m = Marker()
            m.header.frame_id = self.marker_frame
            m.header.stamp = now
            m.ns = 'edge_corners'
            m.id = 1000 + idx
            m.type = Marker.SPHERE
            m.scale.x = m.scale.y = m.scale.z = self.corner_marker_scale
            m.color.r = 0.0; m.color.g = 0.5; m.color.b = 1.0; m.color.a = 1.0
            m.pose.position = GeoPoint(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            ma.markers.append(m)

        # Outline (connect in the provided order and close)
        line = Marker()
        line.header.frame_id = self.marker_frame
        line.header.stamp = now
        line.ns = 'edge_outline'
        line.id = 2000
        line.type = Marker.LINE_STRIP
        line.scale.x = self.line_width
        line.color.r = 1.0; line.color.g = 1.0; line.color.b = 0.0; line.color.a = 1.0

        for p in coords:
            line.points.append(GeoPoint(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        # close loop
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
