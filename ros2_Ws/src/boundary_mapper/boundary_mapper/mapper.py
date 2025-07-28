#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
import random


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
        # publisher
        self.pub = self.create_publisher(String, 'boundary_edges', 10)

    def pose_cb(self, msg: PoseStamped):
        # update current drone altitude
        self.current_z = msg.pose.position.z

    def cloud_callback(self, msg: PointCloud2):
        # convert pointcloud to numpy array
        cloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        pts = []
        for p in cloud:
            r, g, b = float(p['r']), float(p['g']), float(p['b'])
            s = r + g + b
            if s <= 0: continue
            rn, gn = r/s, g/s
            if rn > 0.35 and gn > 0.35 and rn > gn:
                pts.append((p['x'], p['y']))
        if len(pts) < 2:
            self.publish_line([])
            return

        pts2d = np.array(pts)
        lshape = self.detect_lshape(pts2d)
        if lshape:
            A, I, C = lshape
            len1 = np.linalg.norm(A - I)
            len2 = np.linalg.norm(C - I)
            if len1 >= self.min_segment_length and len2 >= self.min_segment_length:
                coords = [f"{p[0]:.3f},{p[1]:.3f},{self.current_z:.3f}" for p in [A, I, C]]
                self.publish_edge(coords)
                return
        line = self.detect_line(pts2d)
        if line is not None:
            P1, P2 = line
            coords = [
                f"{P1[0]:.3f},{P1[1]:.3f},{self.current_z:.3f}",
                f"{P2[0]:.3f},{P2[1]:.3f},{self.current_z:.3f}"
            ]
            self.publish_line(coords)
        else:
            self.publish_line([])

    def detect_lshape(self, pts2d):
        best = None
        best_inliers = 0
        N = len(pts2d)
        for _ in range(2000):
            i, j, k, l = random.sample(range(N), 4)
            P1, P2, P3, P4 = pts2d[i], pts2d[j], pts2d[k], pts2d[l]
            v1, v2 = P2 - P1, P4 - P3
            if np.linalg.norm(v1) < 1e-3 or np.linalg.norm(v2) < 1e-3: continue
            d1 = v1 / np.linalg.norm(v1)
            d2 = v2 / np.linalg.norm(v2)
            if abs(np.dot(d1, d2)) > 0.1: continue
            inliers = sum(
                abs(cross2d(P - P1, d1))<0.05 or abs(cross2d(P - P3, d2))<0.05
                for P in pts2d
            )
            if inliers > best_inliers:
                best_inliers, best = inliers, (P1, d1, P3, d2)
        if not best:
            return None
        P1, d1, P3, d2 = best
        if abs(np.dot(d1, d2)) > 0.05:
            d2 = np.array([-d1[1], d1[0]])
        denom = cross2d(d1, d2)
        if abs(denom) < 1e-6: return None
        t = cross2d(P3 - P1, d2) / denom
        I = P1 + t*d1
        L1 = [P for P in pts2d if abs(cross2d(P - P1, d1))<0.05]
        L2 = [P for P in pts2d if abs(cross2d(P - P3, d2))<0.05]
        if not L1 or not L2: return None
        far1 = max(L1, key=lambda P: abs((P - I).dot(d1)))
        far2 = max(L2, key=lambda P: abs((P - I).dot(d2)))
        return far1, I, far2

    def detect_line(self, pts2d):
        best = None
        best_inliers = 0
        N = len(pts2d)
        for _ in range(2000):
            i, j = random.sample(range(N), 2)
            P1, P2 = pts2d[i], pts2d[j]
            v = P2 - P1
            if np.linalg.norm(v) < 1e-3: continue
            d = v / np.linalg.norm(v)
            inliers = [P for P in pts2d if abs(cross2d(P - P1, d))<0.05]
            if len(inliers) > best_inliers:
                best_inliers, best = len(inliers), (P1, P2)
        if not best: return None
        return best

    def publish_edge(self, coords):
        msg = String()
        msg.data = 'edge:' + ';'.join(coords)
        self.pub.publish(msg)

    def publish_line(self, coords):
        msg = String()
        msg.data = 'line:' + ';'.join(coords)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BoundaryMapper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
