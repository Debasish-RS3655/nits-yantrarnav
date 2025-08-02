#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import PositionTarget, State
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
        self.step = self.get_parameter('step_size').value
        self.slide_tol = self.get_parameter('slide_tol').value

        # state
        self.current_pose = None
        self.armed = False
        self.mode = ''
        self.phase = 0   # 0=forward_x, 1=follow_line, 2=align_edge, 3=return_right, 4=done
        self.line_pts = None  # list of (x, y, z)
        self.edge_pts = None  # list of (x, y, z)
        self.edge_dir = None  # +1 or -1 for y direction on edge alignment
        self.current_target = None

        # subscribers
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, 10)
        self.create_subscription(String, 'boundary_edges', self.edges_cb, 10)

        # publishers
        # self.nav_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.nav_pub = self.create_publisher(PositionTarget, '/dummy_mavros/setpoint_raw/local', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/boundary_navigator_setpoints', 10)
        self.create_timer(0.2, self.tick)

        self.get_logger().info("BoundaryNavigator node started.")

    def state_cb(self, msg: State):
        self.armed = msg.armed
        self.mode = msg.mode

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def edges_cb(self, msg: String):
        if self.current_pose is None:
            self.get_logger().warn("Received boundary_edges but pose is not yet available from MAVROS. Skipping this message.")
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
                self.get_logger().info('Line detected, switching to follow_line phase')

        elif typ == 'edge' and coords:
            self.edge_pts = coords
            if self.phase in (0,1):
                self.phase = 2
                curr_y = self.current_pose.pose.position.y
                iy = coords[1][1]
                self.edge_dir = 1 if iy > curr_y else -1
                self.get_logger().info('Edge detected, switching to align_edge phase')

    def tick(self):
        if not (self.armed and self.mode == 'OFFBOARD' and self.current_pose):
            return
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z

        def reached(tx, ty):
            return np.hypot(tx - x, ty - y) < self.slide_tol

        if self.phase == 0:
            tx, ty = x + self.step, y
            if self.current_target is None or reached(*self.current_target):
                self.current_target = (tx, ty)
                self.send_setpoint(tx, ty, z)

        elif self.phase == 1 and self.line_pts:
            px1, py1, _ = self.line_pts[0]
            px2, py2, _ = self.line_pts[1]
            line_x = (px1 + px2) / 2.0
            tx, ty = line_x, y + self.step
            if self.current_target is None or reached(*self.current_target):
                self.current_target = (tx, ty)
                self.send_setpoint(tx, ty, z)

        elif self.phase == 2 and self.edge_pts:
            ix, iy, _ = self.edge_pts[1]
            tx, ty = ix, y + self.edge_dir * self.step
            if abs(y - iy) < self.slide_tol:
                self.phase = 3
                self.get_logger().info('Aligned to edge, switching to return_right phase')
            elif self.current_target is None or reached(*self.current_target):
                self.current_target = (tx, ty)
                self.send_setpoint(tx, ty, z)

        elif self.phase == 3 and self.edge_pts:
            A = np.array(self.edge_pts[0])
            C = np.array(self.edge_pts[2])
            target_corner = C if self.edge_dir > 0 else A
            tx, ty = x, y - self.edge_dir * self.step
            if abs(y - target_corner[1]) < self.slide_tol:
                self.phase = 4
                self.get_logger().info('Reached opposite edge, navigation done')
            elif self.current_target is None or reached(*self.current_target):
                self.current_target = (tx, ty)
                self.send_setpoint(tx, ty, z)

        # Phase 4: idle

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
        sp.position.x = x
        sp.position.y = y
        sp.position.z = z
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