#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as GeoPoint
import numpy as np
import random

# cross product in 2D

def cross2d(a, b):
    return a[0]*b[1] - a[1]*b[0]

class BoundaryMapper(Node):
    def __init__(self):
        super().__init__('boundary_mapper_py')
        # parameters
        self.declare_parameter('cloud_topic', '/rtabmap/cloud_ground')
        self.declare_parameter('min_segment_length', 0.5)
        topic = self.get_parameter('cloud_topic').value
        self.min_segment_length = self.get_parameter('min_segment_length').value

        # state
        self.current_z = 0.0

        # subscriptions
        self.create_subscription(PointCloud2, topic, self.cloud_callback, 10)
        self.create_subscription(PoseStamped,
                                 '/mavros/local_position/pose',
                                 self.pose_cb, 10)
        # publishers
        self.pub = self.create_publisher(String, 'boundary_edges', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'boundary_markers', 10)
        self.get_logger().info(f"BoundaryMapper started, subscribed to {topic}")

    def pose_cb(self, msg: PoseStamped):
        self.current_z = msg.pose.position.z

    def cloud_callback(self, msg: PointCloud2):
        # manual conversion from PointCloud2
        pts = []
        for x, y, z, r, g, b in point_cloud2.read_points(msg,
                                                       field_names=('x','y','z','r','g','b'),
                                                       skip_nans=True):
            r_f, g_f, b_f = float(r), float(g), float(b)
            s = r_f + g_f + b_f
            if s <= 0:
                continue
            rn, gn = r_f/s, g_f/s
            if rn > 0.35 and gn > 0.35 and rn > gn:
                pts.append((x, y))
        if len(pts) < 2:
            self.publish_line([])
            return

        pts2d = np.array(pts)
        # detect L-shape or line
        lshape = self.detect_lshape(pts2d)
        coords = []
        marker_points = []
        if lshape:
            A, I, C = lshape
            len1 = np.linalg.norm(A - I)
            len2 = np.linalg.norm(C - I)
            if len1 >= self.min_segment_length and len2 >= self.min_segment_length:
                coords = [f"{p[0]:.3f},{p[1]:.3f},{self.current_z:.3f}" for p in [A, I, C]]
                marker_points = [A, I, C]
                self.publish_edge(coords)
        if not coords:
            line = self.detect_line(pts2d)
            if line is not None:
                P1, P2 = line
                coords = [
                    f"{P1[0]:.3f},{P1[1]:.3f},{self.current_z:.3f}",
                    f"{P2[0]:.3f},{P2[1]:.3f},{self.current_z:.3f}" 
                ]
                marker_points = [P1, P2]
                self.publish_line(coords)
        if marker_points:
            self.publish_markers(msg.header.frame_id, msg.header.stamp, marker_points)
    def detect_lshape(self, pts2d):
        # same detection as before
        best = None
        best_inliers = 0
        N = len(pts2d)
        for _ in range(2000):
            i,j,k,l = random.sample(range(N), 4)
            P1,P2,P3,P4 = pts2d[i], pts2d[j], pts2d[k], pts2d[l]
            v1, v2 = P2-P1, P4-P3
            if np.linalg.norm(v1)<1e-3 or np.linalg.norm(v2)<1e-3: continue
            d1, d2 = v1/np.linalg.norm(v1), v2/np.linalg.norm(v2)
            if abs(np.dot(d1,d2))>0.1: continue
            inliers = sum(abs(cross2d(P-P1,d1))<0.05 or abs(cross2d(P-P3,d2))<0.05 for P in pts2d)
            if inliers>best_inliers:
                best_inliers, best = inliers, (P1, P1+d1, P3, P3+d2)
        if not best:
            return None
        A,_,C,_ = best
        # compute intersection I
        d1 = (best[1]-best[0]); d2 = (best[3]-best[2])
        if abs(np.dot(d1,d2))>0.05: d2 = np.array([-d1[1], d1[0]])
        denom = cross2d(d1,d2);
        if abs(denom)<1e-6: return None
        t = cross2d(best[2]-best[0],d2)/denom
        I = best[0] + t*d1
        return A, I, C

    def detect_line(self, pts2d):
        best = None
        best_inliers = 0
        N = len(pts2d)
        for _ in range(2000):
            i,j = random.sample(range(N),2)
            P1,P2 = pts2d[i], pts2d[j]
            v = P2-P1
            if np.linalg.norm(v)<1e-3: continue
            d = v/np.linalg.norm(v)
            inliers = [P for P in pts2d if abs(cross2d(P-P1,d))<0.05]
            if len(inliers)>best_inliers:
                best_inliers, best = len(inliers), (P1,P2)
        return best

    def publish_edge(self, coords):
        msg = String(); msg.data = 'edge:' + ';'.join(coords)
        self.pub.publish(msg)

    def publish_line(self, coords):
        msg = String(); msg.data = 'line:' + ';'.join(coords)
        self.pub.publish(msg)

    def publish_markers(self, frame_id, stamp, points):
        ma = MarkerArray()
        # sphere list for yellow points omitted; only green line
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = 'boundary_line'
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.05
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
        for p in points:
            gp = GeoPoint(x=float(p[0]), y=float(p[1]), z=self.current_z)
            m.points.append(gp)
        ma.markers.append(m)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = BoundaryMapper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
