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
import random, struct

def cross2d(a, b):
    return a[0]*b[1] - a[1]*b[0]

def unpack_rgb(rgb_float):
    """Unpack float32 rgb field into r,g,b floats 0–255."""
    i = struct.unpack('<I', struct.pack('<f', rgb_float))[0]
    r = (i >> 16) & 0xFF
    g = (i >> 8) & 0xFF
    b = i & 0xFF
    return float(r), float(g), float(b)

class BoundaryMapper(Node):
    def __init__(self):
        super().__init__('boundary_mapper_py')
        # Parameters
        self.declare_parameter('cloud_topic', '/rtabmap/cloud_ground')
        self.declare_parameter('min_segment_length', 0.5)
        self.declare_parameter('min_yellow_points', 20)
        self.declare_parameter('min_yellow_density', 5.5)  # points per m²
        self.declare_parameter('outlier_mean_k', 30)
        self.declare_parameter('outlier_stddev_mul', 1.0)

        topic = self.get_parameter('cloud_topic').value
        self.min_segment_length = self.get_parameter('min_segment_length').value
        self.min_yellow_points = self.get_parameter('min_yellow_points').value
        self.min_yellow_density = self.get_parameter('min_yellow_density').value
        self.outlier_mean_k = self.get_parameter('outlier_mean_k').value
        self.outlier_stddev_mul = self.get_parameter('outlier_stddev_mul').value

        self.current_pose = None
        self.current_z = 0.0

        self.create_subscription(PointCloud2, topic, self.cloud_callback, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, 10)

        self.pub = self.create_publisher(String, 'boundary_edges', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'boundary_markers', 10)
        self.get_logger().info(f"BoundaryMapper started, subscribed to {topic}")

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg
        self.current_z = msg.pose.position.z

    def fit_plane(self, xyz):
        """Fit z = ax + by + c using least squares."""
        if len(xyz) < 3:
            return (0,0,self.current_z)
        xyz = np.array(xyz)
        A = np.c_[xyz[:,0], xyz[:,1], np.ones(len(xyz))]
        z = xyz[:,2]
        coeffs, *_ = np.linalg.lstsq(A, z, rcond=None)
        return coeffs  # a, b, c

    def cloud_callback(self, msg: PointCloud2):
        pts = []
        xyzrgb = []

        # Extract yellow points
        field_names = [f.name for f in msg.fields]
        if "rgb" in field_names or "rgba" in field_names:
            use = "rgba" if "rgba" in field_names else "rgb"
            for x,y,z,rgb in point_cloud2.read_points(msg, field_names=("x","y","z",use), skip_nans=True):
                r,g,b = unpack_rgb(rgb)
                s = r+g+b
                if s == 0: continue
                rn, gn = r/s, g/s
                if rn > 0.351 and gn > 0.351 and rn > gn:
                    pts.append((x,y))
                    xyzrgb.append((x,y,z))
        else:
            for x,y,z,r,g,b in point_cloud2.read_points(msg, field_names=("x","y","z","r","g","b"), skip_nans=True):
                s = r+g+b
                if s == 0: continue
                rn, gn = r/s, g/s
                if rn > 0.351 and gn > 0.351 and rn > gn:
                    pts.append((x,y))
                    xyzrgb.append((x,y,z))

        if len(pts) < self.min_yellow_points:
            self.get_logger().warn(f"Too few yellow points: {len(pts)} < {self.min_yellow_points}.")
            self.publish_line([])
            return

        # Compute density
        pts_np = np.array(pts)
        x_min, y_min = np.min(pts_np, axis=0)
        x_max, y_max = np.max(pts_np, axis=0)
        area = (x_max - x_min) * (y_max - y_min)
        density = len(pts) / area if area > 0 else 0.0

        if density < self.min_yellow_density:
            self.get_logger().warn(f"Yellow point density too low: {density:.2f} < {self.min_yellow_density}.")
            self.publish_line([])
            return

        coeffs = self.fit_plane(xyzrgb)
        def z_on_plane(x,y):
            return coeffs[0]*x + coeffs[1]*y + coeffs[2]

        coords = []
        marker_points = []

        # Try L-shape detection
        lshape = self.detect_lshape(pts_np)
        if lshape:
            A, I, C = lshape
            len1 = np.linalg.norm(A - I)
            len2 = np.linalg.norm(C - I)
            if len1 >= self.min_segment_length and len2 >= self.min_segment_length:
                marker_points = [A, I, C]
                coords = [f"{p[0]:.3f},{p[1]:.3f},{z_on_plane(p[0],p[1]):.3f}" for p in marker_points]
                self.publish_edge(coords)

        if not coords:
            # fallback: try a line
            line = self.detect_line(pts_np)
            if line:
                P1, P2 = line
                marker_points = [P1, P2]
                coords = [f"{P1[0]:.3f},{P1[1]:.3f},{z_on_plane(P1[0],P1[1]):.3f}",
                          f"{P2[0]:.3f},{P2[1]:.3f},{z_on_plane(P2[0],P2[1]):.3f}"]
                self.publish_line(coords)

        if marker_points:
            self.publish_markers(msg.header.frame_id or "map", msg.header.stamp, marker_points, z_on_plane, xyzrgb)

    def detect_lshape(self, pts2d):
        best = None
        best_inliers = 0
        N = len(pts2d)
        for _ in range(2000):
            i,j,k,l = random.sample(range(N),4)
            P1,P2,P3,P4 = pts2d[i],pts2d[j],pts2d[k],pts2d[l]
            v1,v2 = P2-P1, P4-P3
            if np.linalg.norm(v1)<1e-3 or np.linalg.norm(v2)<1e-3: continue
            d1,d2 = v1/np.linalg.norm(v1), v2/np.linalg.norm(v2)
            if abs(np.dot(d1,d2))>0.1: continue
            in1 = [P for P in pts2d if abs(cross2d(P-P1,d1))<0.05]
            in2 = [P for P in pts2d if abs(cross2d(P-P3,d2))<0.05]
            inliers = len(set(map(tuple,in1)) | set(map(tuple,in2)))
            if inliers > best_inliers and in1 and in2:
                best_inliers = inliers
                best = (P1,d1,P3,d2,np.array(in1),np.array(in2))
        if not best:
            return None
        P1,d1,P3,d2,in1,in2 = best
        denom = cross2d(d1,d2)
        if abs(denom)<1e-6: return None
        t = cross2d(P3-P1,d2)/denom
        I = P1 + t*d1
        def extremal(inliers,I,d):
            projs = np.abs((inliers - I) @ d)
            return inliers[np.argmax(projs)]
        A = extremal(in1,I,d1)
        C = extremal(in2,I,d2)
        return A,I,C

    def detect_line(self, pts2d):
        best = None
        best_inliers = 0
        N = len(pts2d)
        for _ in range(2000):
            i,j = random.sample(range(N),2)
            P1,P2 = pts2d[i],pts2d[j]
            v = P2-P1
            if np.linalg.norm(v)<1e-3: continue
            d = v/np.linalg.norm(v)
            inliers = [P for P in pts2d if abs(cross2d(P-P1,d))<0.05]
            if len(inliers)>best_inliers:
                best_inliers = len(inliers)
                inliers = np.array(inliers)
                projs = (inliers-P1) @ d
                Pmin,Pmax = inliers[np.argmin(projs)], inliers[np.argmax(projs)]
                best = (Pmin,Pmax)
        return best

    def publish_edge(self, coords):
        msg = String(); msg.data = 'edge:' + ';'.join(coords)
        self.pub.publish(msg)

    def publish_line(self, coords):
        msg = String(); msg.data = 'line:' + ';'.join(coords)
        self.pub.publish(msg)

    def publish_markers(self, frame_id, stamp, points, z_func, xyzrgb):
        ma = MarkerArray()

        # Yellow points
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = "yellow_points"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.scale.x = m.scale.y = m.scale.z = 0.03
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
        for (x,y,z) in xyzrgb:
            gp = GeoPoint(x=float(x),y=float(y),z=float(z))
            m.points.append(gp)
        ma.markers.append(m)

        # Boundary line
        l = Marker()
        l.header.frame_id = frame_id
        l.header.stamp = stamp
        l.ns = "boundary_line"
        l.id = 1
        l.type = Marker.LINE_STRIP
        l.scale.x = 0.05
        l.color.r = 0.0; l.color.g = 1.0; l.color.b = 0.0; l.color.a = 1.0
        for p in points:
            gp = GeoPoint(x=float(p[0]), y=float(p[1]), z=z_func(p[0],p[1]))
            l.points.append(gp)
        ma.markers.append(l)

        self.marker_pub.publish(ma)

def main():
    rclpy.init()
    node = BoundaryMapper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
