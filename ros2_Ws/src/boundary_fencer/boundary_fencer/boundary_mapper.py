import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from boundary_fencer_pkg.msg import FenceCorners
from geometry_msgs.msg import Point, Vector3
import pcl_conversions
import pcl
import numpy as np
import random

def cross2d(a, b):
    return a[0]*b[1] - a[1]*b[0]

class BoundaryMapper(Node):
    def __init__(self):
        super().__init__('boundary_mapper_py')
        self.declare_parameter('cloud_topic', '/rtabmap/cloud_ground')
        topic = self.get_parameter('cloud_topic').value

        self.sub = self.create_subscription(
            PointCloud2, topic, self.cloud_callback, 10)
        self.pub = self.create_publisher(FenceCorners, 'fence_corners', 10)

    def cloud_callback(self, msg: PointCloud2):
        # Convert to PCL and extract yellow points
        cloud = pcl.PointCloud_PointXYZRGB()
        pcl_conversions.point_cloud2_to_pcl(msg, cloud)
        pts = []
        for p in cloud:
            s = p.r + p.g + p.b
            if s == 0: continue
            rn, gn = p.r/s, p.g/s
            if rn > 0.35 and gn > 0.35 and rn > gn:
                pts.append((p.x, p.y, p.z))
        if len(pts) < 4:
            self.publish_failure('edge', msg.header)
            return

        # 2D projection
        pts2d = np.array([[x,y] for x,y,_ in pts])

        # RANSAC for two perpendicular lines
        best = None
        max_inliers = 0
        for _ in range(2000):
            a,b,c,d = random.sample(range(len(pts2d)), 4)
            v1 = pts2d[b]-pts2d[a]
            v2 = pts2d[d]-pts2d[c]
            if np.linalg.norm(v1)<1e-3 or np.linalg.norm(v2)<1e-3:
                continue
            d1 = v1/np.linalg.norm(v1)
            d2 = v2/np.linalg.norm(v2)
            if abs(np.dot(d1,d2))>0.1:
                continue
            # count inliers
            inliers = 0
            for p in pts2d:
                dist1 = abs(cross2d(p-pts2d[a], d1))
                dist2 = abs(cross2d(p-pts2d[c], d2))
                if dist1<0.05 or dist2<0.05:
                    inliers += 1
            if inliers > max_inliers:
                max_inliers = inliers
                best = ((pts2d[a], d1), (pts2d[c], d2))

        if best is None:
            self.publish_failure('line', msg.header)
            return

        (P1, d1), (P2, d2) = best
        # ensure perpendicular
        if abs(np.dot(d1, d2))>0.05:
            d2 = np.array([-d1[1], d1[0]])

        # intersection
        delta = P2 - P1
        denom = cross2d(d1, d2)
        if abs(denom)<1e-6:
            self.publish_failure('line', msg.header)
            return
        t = cross2d(delta, d2)/denom
        I = P1 + t*d1

        # extract corner points
        def max_proj(pts_line, dir_vec):
            return max(pts_line, key=lambda p: abs(np.dot(p-I, dir_vec)))

        L1_in = [p for p in pts2d if abs(cross2d(p-P1, d1))<0.05]
        L2_in = [p for p in pts2d if abs(cross2d(p-P2, d2))<0.05]
        if not L1_in or not L2_in:
            self.publish_failure('edge', msg.header)
            return

        A = max_proj(L1_in, d1); B = I; C = max_proj(L2_in, d2)

        # plane fit for Z = ax + by + c
        A_mat = np.c_[np.array([[x,y,1] for x,y,_ in pts]),]
        b_vec = np.array([z for _,_,z in pts])
        a, b, c0 = np.linalg.lstsq(A_mat, b_vec, rcond=None)[0]
        def compute_z(x,y): return a*x + b*y + c0

        # prepare message
        msg_out = FenceCorners()
        msg_out.header = msg.header
        for P in [A,B,C]:
            p = Point(x=P[0], y=P[1], z=float(compute_z(P[0], P[1])))
            msg_out.corners.append(p)
        dir_unit = d1/np.linalg.norm(d1)
        msg_out.direction = Vector3(x=dir_unit[0], y=dir_unit[1], z=0.0)
        msg_out.target_waypoint = msg_out.corners[0]
        msg_out.status = 'ok'
        self.pub.publish(msg_out)

    def publish_failure(self, stat, header):
        msg_out = FenceCorners()
        msg_out.header = header
        msg_out.status = stat
        self.pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryMapper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
