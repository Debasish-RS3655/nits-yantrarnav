#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import PositionTarget, State
from boundary_mapper.msg import FenceCorners
import ros_numpy
import numpy as np

class BoundaryNavigator(Node):
    def __init__(self):
        super().__init__('boundary_navigator')
        # parameters
        self.declare_parameter('step_size', 0.5)        # meters per tick
        self.declare_parameter('detect_thresh', 4)      # min yellow points
        self.declare_parameter('line_tol', 0.05)        # 5 cm to line
        self.declare_parameter('axis', 'x')             # move along 'x' then reverse

        self.state_sub = self.create_subscription(State,
                             '/mavros/state', self.state_cb, 10)
        self.pose_sub  = self.create_subscription(PoseStamped,
                             '/mavros/local_position/pose', self.pose_cb, 10)
        self.pc_sub    = self.create_subscription(PointCloud2,
                             '/rtabmap/cloud_ground', self.pc_cb, 10)

        self.nav_pub   = self.create_publisher(PositionTarget,
                             '/mavros/setpoint_raw/local', 10)
        self.corners_pub = self.create_publisher(FenceCorners,
                             'fence_corners', 10)

        self.current_pose = None
        self.armed = False
        self.mode = ""
        self.step = float(self.get_parameter('step_size').value)
        self.detect_thresh = int(self.get_parameter('detect_thresh').value)
        self.line_tol = float(self.get_parameter('line_tol').value)
        self.axis = self.get_parameter('axis').value

        self.phase = 0   # 0=search1,1=slide1,2=search2,3=done
        self.first_pt = None                # first and second egdes
        self.second_pt = None
        self.timer = self.create_timer(0.2, self.tick)

    def state_cb(self, msg: State):
        self.armed = msg.armed
        self.mode  = msg.mode

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def pc_cb(self, msg: PointCloud2):
        # extract yellow ground points in XY
        arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        pts = np.array([[p['x'],p['y']] 
                        for p in arr
                        if (p['r']+p['g']+p['b'])>0
                        and (p['r']/(p['r']+p['g']+p['b'])>0.35)
                        and (p['g']/(p['r']+p['g']+p['b'])>0.35)])
        self.last_yellow = pts

    def tick(self):
        if not (self.armed and self.mode=='OFFBOARD' and self.current_pose): 
            return

        x, y, z = (self.current_pose.pose.position.x,
                   self.current_pose.pose.position.y,
                   self.current_pose.pose.position.z)

        # convenience: move along +axis or –axis
        dx, dy = (self.step,0) if self.axis=='x' else (0,self.step)

        if self.phase == 0:
            # search first boundary
            if hasattr(self, 'last_yellow') and len(self.last_yellow)>=self.detect_thresh:
                # project onto axis perpendicular
                perp_idx = 1 if self.axis=='x' else 0
                pts_proj = self.last_yellow[:, perp_idx]
                mid = float((pts_proj.min()+pts_proj.max())/2.0)
                # record first corner line midpoint
                self.first_pt = (mid, y) if self.axis=='x' else (x, mid)
                self.get_logger().info(f"Found 1st edge at {self.first_pt}")
                self.phase = 1
            else:
                # step forward
                self.send_setpoint(x+dx, y+dy, -3.0)
        elif self.phase == 1:
            # slide parallel to boundary: freeze perp coord, move along axis
            perp_idx = 1 if self.axis=='x' else 0
            target = list(self.first_pt)
            # moving along axis idx
            move_idx = 0 if self.axis=='x' else 1
            pos = [x,y]
            if abs(pos[perp_idx] - target[perp_idx])>0.1:
                # adjust perp to line
                pos[perp_idx] += (target[perp_idx] - pos[perp_idx]) * 0.5
            else:
                # go to next phase
                self.get_logger().info("Aligned parallel to 1st edge")
                self.phase = 2
            self.send_setpoint(pos[0], pos[1], -3.0)
        elif self.phase == 2:
            # search opposite boundary: reverse step
            dx, dy = -dx, -dy
            if hasattr(self, 'last_yellow') and len(self.last_yellow)>=self.detect_thresh:
                perp_idx = 1 if self.axis=='x' else 0
                pts_proj = self.last_yellow[:, perp_idx]
                mid = float((pts_proj.min()+pts_proj.max())/2.0)
                self.second_pt = (mid, y) if self.axis=='x' else (x, mid)
                self.get_logger().info(f"Found 2nd edge at {self.second_pt}")
                self.phase = 3
            else:
                self.send_setpoint(x+dx, y+dy, -3.0)
        elif self.phase == 3:
            # done — publish corners
            msg = FenceCorners()
            msg.header.stamp = self.get_clock().now().to_msg()
            # fill in (x,y,z) for each corner: first_pt & second_pt
            for px,py in [self.first_pt, self.second_pt]:
                p = Point(x=px, y=py, z=-3.0)
                msg.corners.append(p)
            msg.direction.x = 1.0 if self.axis=='x' else 0.0
            msg.direction.y = 0.0 if self.axis=='x' else 1.0
            msg.direction.z = 0.0
            msg.target_waypoint = msg.corners[0]
            msg.status = 'ok'
            self.corners_pub.publish(msg)
            self.get_logger().info("Published fence corners")
            self.phase = 4

    def send_setpoint(self, x, y, z):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_VX |
                        PositionTarget.IGNORE_VY |
                        PositionTarget.IGNORE_VZ |
                        PositionTarget.IGNORE_AFX |
                        PositionTarget.IGNORE_AFY |
                        PositionTarget.IGNORE_AFZ |
                        PositionTarget.IGNORE_YAW |
                        PositionTarget.IGNORE_YAW_RATE)
        sp.position.x = x
        sp.position.y = y
        sp.position.z = z
        self.nav_pub.publish(sp)


def main():
    rclpy.init()
    node = BoundaryNavigator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
