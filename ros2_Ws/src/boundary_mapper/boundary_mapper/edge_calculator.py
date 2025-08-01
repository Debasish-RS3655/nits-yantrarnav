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
        self.declare_parameter('tol', 0.5)                        # kept for compatibility; unused
        self.declare_parameter('min_edge_separation', 7.5)        # meters between saved corners (I points)
        self.declare_parameter('marker_frame', 'map')
        self.declare_parameter('corner_marker_scale', 0.25)       # meters (sphere diameter)
        self.declare_parameter('line_width', 0.07)                # meters
        # Rectangle dimension rules
        self.declare_parameter('rect_long_side_len', 12.0)        # meters
        self.declare_parameter('rect_short_side_len', 9.0)        # meters
        self.declare_parameter('rect_side_len_tol', 1.0)          # meters tolerance

        self.tol = self.get_parameter('tol').value
        self.min_edge_separation = float(self.get_parameter('min_edge_separation').value)
        self.marker_frame = self.get_parameter('marker_frame').value
        self.corner_marker_scale = float(self.get_parameter('corner_marker_scale').value)
        self.line_width = float(self.get_parameter('line_width').value)
        self.rect_long = float(self.get_parameter('rect_long_side_len').value)
        self.rect_short = float(self.get_parameter('rect_short_side_len').value)
        self.rect_tol = float(self.get_parameter('rect_side_len_tol').value)

        # State
        # Each saved edge is a dict: {'A':np3, 'I':np3, 'C':np3}
        self.saved_edges = []
        self.initial_corner_I = None
        self.corner_count = 0      # for marker IDs

        # I/O
        self.create_subscription(String, 'boundary_edges', self.cb, 10)
        self.pub_rect = self.create_publisher(String, 'fence_rectangle', 10)
        self.pub_saved_corner = self.create_publisher(String, 'saved_corner', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'edge_markers', 10)

        self.get_logger().info("EdgeCalculator node has started.")

    # ---------------- Parsing ----------------

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

    # ---------------- Callbacks ----------------

    def cb(self, msg: String):
        parsed = self.parse_edge(msg.data)
        if not parsed:
            return

        A, I, C = parsed

        # Gate by separation from the first saved corner (I point)
        if self.initial_corner_I is None:
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

        # If we now have two saved edges, compute the 12x9 rectangle and visualize/publish
        if len(self.saved_edges) == 2:
            I1 = self.saved_edges[0]['I']
            I2 = self.saved_edges[1]['I']
            A1, C1 = self.saved_edges[0]['A'], self.saved_edges[0]['C']
            A2, C2 = self.saved_edges[1]['A'], self.saved_edges[1]['C']

            # Decide which dimension the red-to-red span represents
            v = I2[:2] - I1[:2]                        # XY
            L = np.linalg.norm(v)
            if L < 1e-6:
                self.get_logger().warn("Corners too close; cannot build rectangle.")
                self._reset_pair_state()
                return

            if abs(L - self.rect_long) <= self.rect_tol:
                along_len = self.rect_long
                perp_len = self.rect_short
            elif abs(L - self.rect_short) <= self.rect_tol:
                along_len = self.rect_short
                perp_len = self.rect_long
            else:
                # If neither close, choose the nearest intended side and log
                if abs(L - self.rect_long) < abs(L - self.rect_short):
                    along_len, perp_len = self.rect_long, self.rect_short
                else:
                    along_len, perp_len = self.rect_short, self.rect_long
                self.get_logger().warn(
                    f"Corner spacing {L:.2f} m not within tol {self.rect_tol:.2f} of "
                    f"{self.rect_long} or {self.rect_short}. Using nearest: along={along_len}, perp={perp_len}."
                )

            # Build a robust perpendicular using corner arm directions
            # For each corner, identify which arm is parallel to v and which is perpendicular
            def unit2(w):
                n = np.linalg.norm(w)
                return w / n if n > 1e-9 else w

            v_hat = unit2(v)

            def corner_perp(I, A, C):
                dA = unit2((A[:2] - I[:2]))
                dC = unit2((C[:2] - I[:2]))
                # choose the arm most parallel to v, the other is perpendicular
                if abs(dA @ v_hat) >= abs(dC @ v_hat):
                    return dC
                else:
                    return dA

            p1 = corner_perp(I1, A1, C1)
            p2 = corner_perp(I2, A2, C2)
            p = p1 + p2
            if np.linalg.norm(p) < 1e-6:
                # fall back to a canonical perpendicular to v_hat
                p = np.array([-v_hat[1], v_hat[0]])
            p_hat = unit2(p)

            # Construct the other two rectangle corners at 'perp_len' from each red corner
            J1 = I1.copy()
            J2 = I2.copy()
            J1[:2] = I1[:2] + p_hat * perp_len
            J2[:2] = I2[:2] + p_hat * perp_len
            # keep Z as original corner Z (or average, if you prefer)

            # Order for visualization/output (loop): I1 -> I2 -> J2 -> J1 -> I1
            rect_corners = [I1, I2, J2, J1]

            # Publish: 4 rectangle corners (computed), as "x,y,z" comma-separated
            pairs = [f"{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}" for p in rect_corners]
            out = String()
            out.data = ",".join(pairs)
            self.pub_rect.publish(out)
            self.get_logger().info(f"Published rectangle (12x9 logic): {out.data}")

            # Visualize: blue markers at these four corners + polyline
            self.visualize_rectangle(rect_corners)

            # Reset to seek a new pair next time
            self._reset_pair_state()

    # ---------------- Helpers ----------------

    def _reset_pair_state(self):
        self.saved_edges.clear()
        self.initial_corner_I = None

    def on_corner_saved(self, I):
        """Publish the saved corner coordinate and log."""
        msg = String()
        msg.data = f"{I[0]:.3f},{I[1]:.3f},{I[2]:.3f}"
        self.pub_saved_corner.publish(msg)
        self.get_logger().info(f"Saved corner: {msg.data}")

    # ---------------- Visualization ----------------

    def visualize_corners(self, corners_np_list):
        """Add corner spheres for each saved corner received (red)."""
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

    def visualize_rectangle(self, corners):
        """
        corners ordered as [I1, I2, J2, J1]; draw spheres at corners and a polyline around them.
        """
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Corner markers (blue)
        for idx, p in enumerate(corners):
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

        # Outline (connect and close)
        line = Marker()
        line.header.frame_id = self.marker_frame
        line.header.stamp = now
        line.ns = 'edge_outline'
        line.id = 2000
        line.type = Marker.LINE_STRIP
        line.scale.x = self.line_width
        line.color.r = 1.0; line.color.g = 1.0; line.color.b = 0.0; line.color.a = 1.0

        for p in corners:
            line.points.append(GeoPoint(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        # close loop
        line.points.append(GeoPoint(x=float(corners[0][0]), y=float(corners[0][1]), z=float(corners[0][2])))
        ma.markers.append(line)

        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = EdgeCalculator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
