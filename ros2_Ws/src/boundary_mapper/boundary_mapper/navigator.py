#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

def cross2d(a, b):
    return a[0]*b[1] - a[1]*b[0]

class BoundaryNavigator(Node):
    def __init__(self):
        super().__init__('boundary_navigator_py')
        # parameters
        self.declare_parameter('step_size', 0.5)
        self.declare_parameter('slide_tol', 0.1)
        self.declare_parameter('min_takeoff_height', 1.0)  # z gate for first setpoint
        self.declare_parameter('arrival_tol', 1.0)         # meters to trigger next setpoint

        self.step = float(self.get_parameter('step_size').value)
        self.slide_tol = float(self.get_parameter('slide_tol').value)
        self.min_takeoff_height = float(self.get_parameter('min_takeoff_height').value)
        self.arrival_tol = float(self.get_parameter('arrival_tol').value)

        # state
        self.current_pose = None
        self.phase = 0   # 0=forward_x, 1=follow_line, 2=align_edge, 3=return_right, 4=done
        self.line_pts = None
        self.edge_pts = None
        self.edge_dir = None
        self.current_target = None
        self.last_setpoint = None  # (x,y,z)
        self.takeoff_gate_open = False
        self._takeoff_log_once = False

        # subscribers (pose kept; no MAVROS state / no ACK)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.pose_cb, qos_profile)
        self.create_subscription(String, 'boundary_edges', self.edges_cb, 10)

        # publishers
        self.nav_pub = self.create_publisher(PositionTarget, '/dummy_mavros/setpoint_raw/local', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/boundary_navigator_setpoints', 10)

        # 5 Hz
        self.create_timer(0.2, self.tick)
        self.get_logger().info("BoundaryNavigator started (1 m takeoff gate + 1 m proximity).")

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg
        if not self.takeoff_gate_open:
            z = self.current_pose.pose.position.z
            if z >= self.min_takeoff_height:
                self.takeoff_gate_open = True
                self.get_logger().info(f"Min takeoff height reached (z={z:.2f} m). Starting.")
            elif not self._takeoff_log_once:
                self._takeoff_log_once = True
                self.get_logger().info(f"Waiting for z ≥ {self.min_takeoff_height} m before first setpoint.")

    def edges_cb(self, msg: String):
        if self.current_pose is None:
            self.get_logger().warn("Received boundary_edges but pose not yet available. Skipping.")
            return
        data = msg.data.split(':')
        typ = data[0]
        coords = []
        if len(data) > 1 and data[1]:
            coords = [tuple(map(float, pt.split(','))) for pt in data[1].split(';')]

        if typ == 'line' and coords:
            self.line_pts = coords
            if self.phase == 0:
                self.phase = 1
                self.get_logger().info('Line detected → follow_line')
        elif typ == 'edge' and coords:
            self.edge_pts = coords
            if self.phase in (0, 1):
                self.phase = 2
                curr_y = self.current_pose.pose.position.y
                iy = coords[1][1]
                self.edge_dir = 1 if iy > curr_y else -1
                self.get_logger().info('Edge detected → align_edge')

    def tick(self):
        if self.current_pose is None or self.phase == 4:
            return
        if not self.takeoff_gate_open:
            return

        x = float(self.current_pose.pose.position.x)
        y = float(self.current_pose.pose.position.y)
        z = float(self.current_pose.pose.position.z)

        # If we have an active setpoint, keep sending it until we are within arrival_tol
        if self.last_setpoint is not None:
            lx, ly, lz = self.last_setpoint
            self.send_setpoint(lx, ly, lz)
            dx, dy, dz = lx - x, ly - y, lz - z
            dist = np.sqrt(dx*dx + dy*dy + dz*dz)  # 3D distance
            if dist > self.arrival_tol:
                return  # not yet reached → keep current setpoint
            # reached → clear and compute next below
            self.last_setpoint = None

        # Compute and send the next target now
        if self.phase == 0:
            tx, ty, tz = x + self.step, y, z
            self.current_target = (tx, ty)
            self.set_and_send(tx, ty, tz)

        elif self.phase == 1 and self.line_pts:
            px1, py1, _ = self.line_pts[0]
            px2, py2, _ = self.line_pts[1]
            line_x = (px1 + px2) / 2.0
            tx, ty, tz = line_x, y + self.step, z
            self.current_target = (tx, ty)
            self.set_and_send(tx, ty, tz)

        elif self.phase == 2 and self.edge_pts:
            ix, iy, _ = self.edge_pts[1]
            if abs(y - iy) < self.slide_tol:
                self.phase = 3
                self.get_logger().info('Aligned to edge → return_right')
                return
            tx, ty, tz = ix, y + self.edge_dir * self.step, z
            self.current_target = (tx, ty)
            self.set_and_send(tx, ty, tz)

        elif self.phase == 3 and self.edge_pts:
            A = np.array(self.edge_pts[0])
            C = np.array(self.edge_pts[2])
            target_corner = C if self.edge_dir > 0 else A
            if abs(y - float(target_corner[1])) < self.slide_tol:
                self.phase = 4
                self.get_logger().info('Reached opposite edge → done')
                return
            tx, ty, tz = x, y - self.edge_dir * self.step, z
            self.current_target = (tx, ty)
            self.set_and_send(tx, ty, tz)

    def set_and_send(self, x, y, z):
        self.last_setpoint = (float(x), float(y), float(z))
        self.send_setpoint(x, y, z)

    def send_setpoint(self, x, y, z):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )
        sp.position.x = float(x)
        sp.position.y = float(y)
        sp.position.z = float(z)
        self.nav_pub.publish(sp)
        self.publish_setpoint_marker(x, y, z)

    def publish_setpoint_marker(self, x, y, z):
        ma = MarkerArray()
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = (
            self.current_pose.header.frame_id
            if self.current_pose and self.current_pose.header.frame_id
            else 'map'
        )
        m.ns = 'boundary_navigator_setpoints'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        ma.markers.append(m)
        self.viz_pub.publish(ma)

def main():
    rclpy.init()
    node = BoundaryNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
