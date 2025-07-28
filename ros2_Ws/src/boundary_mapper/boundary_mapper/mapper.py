#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from boundary_mapper.msg import FenceCorners
from geometry_msgs.msg import Point, Vector3
import ros_numpy
import numpy as np
import random

def cross2d(a, b):
    return a[0]*b[1] - a[1]*b[0]

class BoundaryMapper(Node):
    def __init__(self):
        super().__init__('boundary_mapper_py')
        self.declare_parameter('cloud_topic', '/rtabmap/cloud_ground')
        topic = self.get_parameter('cloud_topic').value        
        self.sub = self.create_subscription(PointCloud2, topic, self.cloud_callback, 10)
        self.pub = self.create_publisher(FenceCorners, 'fence_corners', 10)

    def cloud_callback(self, msg: PointCloud2):
        cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        pts = []
        for p in cloud_arr:
            r, g, b = p['r'], p['g'], p['b']
            s = float(r + g + b)
            if s == 0.0:
                continue
            rn, gn = r/s, g/s
            if rn > 0.35 and gn > 0.35 and rn > gn:
                pts.append((p['x'], p['y'], p['z']))
        if len(pts) < 4:
            return self._publish_status(msg.header, 'edge')

        pts2d = np.array([[x, y] for x, y, _ in pts])
        best, best_inliers = None, 0
        N = len(pts2d)
        for _ in range(2000):
            i, a, b, c = random.sample(range(N), 4)
            P1, P2, P3, P4 = pts2d[i], pts2d[a], pts2d[b], pts2d[c]
            v1, v2 = P2 - P1, P4 - P3
            if np.linalg.norm(v1) < 1e-3 or np.linalg.norm(v2) < 1e-3:
                continue
            d1 = v1 / np.linalg.norm(v1)
            d2 = v2 / np.linalg.norm(v2)
            if abs(np.dot(d1, d2)) > 0.1:
                continue
            inliers = sum(
                abs(cross2d(P - P1, d1)) < 0.05 or abs(cross2d(P - P3, d2)) < 0.05
                for P in pts2d
            )
            if inliers > best_inliers:
                best_inliers, best = inliers, (P1, d1, P3, d2)
        if best is None:
            return self._publish_status(msg.header, 'line')

        P1, d1, P3, d2 = best
        if abs(np.dot(d1, d2)) > 0.05:
            d2 = np.array([-d1[1], d1[0]])
        delta = P3 - P1
        denom = cross2d(d1, d2)
        if abs(denom) < 1e-6:
            return self._publish_status(msg.header, 'line')
        t = cross2d(delta, d2) / denom
        I = P1 + t * d1

        L1 = [P for P in pts2d if abs(cross2d(P - P1, d1)) < 0.05]
        L2 = [P for P in pts2d if abs(cross2d(P - P3, d2)) < 0.05]
        if not L1 or not L2:
            return self._publish_status(msg.header, 'edge')
        far1 = max(L1, key=lambda P: abs(np.dot(P - I, d1)))
        far2 = max(L2, key=lambda P: abs(np.dot(P - I, d2)))

        # fit plane z = ax + by + c
        A = np.array([[x, y, 1.0] for x, y, z in pts])
        b = np.array([z for x, y, z in pts])
        a, b0, c0 = np.linalg.lstsq(A, b, rcond=None)[0]
        compute_z = lambda x, y: a*x + b0*y + c0

        out = FenceCorners()
        out.header = msg.header
        for P in (far1, I, far2):
            pt = Point(x=float(P[0]), y=float(P[1]), z=float(compute_z(P[0], P[1])))
            out.corners.append(pt)
        dir2d = I - far1
        dir2d /= np.linalg.norm(dir2d)
        out.direction = Vector3(x=float(dir2d[0]), y=float(dir2d[1]), z=0.0)
        out.target_waypoint = out.corners[0]
        out.status = 'ok'
        self.pub.publish(out)

    def _publish_status(self, header, st):
        m = FenceCorners()
        m.header = header
        m.status = st
        self.pub.publish(m)


def main():
    rclpy.init()
    node = BoundaryMapper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()