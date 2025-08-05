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

# QoS imports
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

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
        self.declare_parameter('min_yellow_density', 5.0)  # points per m²
        self.declare_parameter('outlier_mean_k', 30)
        self.declare_parameter('outlier_stddev_mul', 1.0)
        # Arm equality thresholds (meters)
        self.declare_parameter('l_arm_equal_threshold', 4.0)
        self.declare_parameter('l_long_edge_min_length', 8.0)
        self.declare_parameter('l_long_edge_equal_threshold', 11.5)
        # Two-corner (multi-edge) controls
        self.declare_parameter('detect_two_edges', True)
        self.declare_parameter('second_edge_exclusion_radius', 2.0)
        # multi-edge validity
        self.declare_parameter('multi_edge_len_match_tol', 2.0)
        self.declare_parameter('multi_edge_parallel_cos_min', 0.85)

        # ---- NEW corner sanity params (only used in detect_lshape) ----
        # Legs must be at least this long (m), gaps along each leg must be <= this (m),
        # and the density ratio between the two legs must be below this.
        self.declare_parameter('corner_min_leg', 1.0)
        self.declare_parameter('corner_max_gap', 0.6)
        self.declare_parameter('edge_density_ratio_max', 1.6)
        # ----------------------------------------------------------------

        topic = self.get_parameter('cloud_topic').value
        self.min_segment_length = self.get_parameter('min_segment_length').value
        self.min_yellow_points = self.get_parameter('min_yellow_points').value
        self.min_yellow_density = self.get_parameter('min_yellow_density').value
        self.outlier_mean_k = self.get_parameter('outlier_mean_k').value
        self.outlier_stddev_mul = self.get_parameter('outlier_stddev_mul').value

        self.l_arm_equal_threshold = self.get_parameter('l_arm_equal_threshold').value
        self.l_long_edge_min_length = self.get_parameter('l_long_edge_min_length').value
        self.l_long_edge_equal_threshold = self.get_parameter('l_long_edge_equal_threshold').value

        self.detect_two_edges = self.get_parameter('detect_two_edges').value
        self.second_edge_exclusion_radius = self.get_parameter('second_edge_exclusion_radius').value

        self.multi_edge_len_match_tol = self.get_parameter('multi_edge_len_match_tol').value
        self.multi_edge_parallel_cos_min = self.get_parameter('multi_edge_parallel_cos_min').value

        # read new corner params
        self.corner_min_leg = self.get_parameter('corner_min_leg').value
        self.corner_max_gap = self.get_parameter('corner_max_gap').value
        self.edge_density_ratio_max = self.get_parameter('edge_density_ratio_max').value

        self.current_pose = None
        self.current_z = 0.0

        # Cloud subscription
        self.create_subscription(PointCloud2, topic, self.cloud_callback, 10)

        # Pose subscription with BEST_EFFORT QoS (fix QoS mismatch with MAVROS)
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, pose_qos)

        self.pub = self.create_publisher(String, 'boundary_edges', 10)

        # Marker publisher with transient QoS to prevent blinking
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(MarkerArray, 'boundary_markers', marker_qos)

        self.get_logger().info(f"BoundaryMapper started, subscribed to {topic}")

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg
        self.current_z = msg.pose.position.z

    def fit_plane(self, xyz):
        if len(xyz) < 3:
            return (0,0,self.current_z)
        xyz = np.array(xyz)
        A = np.c_[xyz[:,0], xyz[:,1], np.ones(len(xyz))]
        z = xyz[:,2]
        coeffs, *_ = np.linalg.lstsq(A, z, rcond=None)
        return coeffs

    def edge_accepts(self, len1, len2):
        if len1 < self.min_segment_length or len2 < self.min_segment_length:
            return False
        longer = max(len1, len2)
        diff = abs(len1 - len2)
        if diff <= self.l_arm_equal_threshold:
            return True
        if longer >= self.l_long_edge_min_length and diff <= self.l_long_edge_equal_threshold:
            return True
        return False

    def validate_two_edges(self, A1, I1, C1, A2, I2, C2):
        v1a = A1 - I1; v1c = C1 - I1
        v2a = A2 - I2; v2c = C2 - I2
        def norm(v): return np.linalg.norm(v)
        def unit(v):
            n = norm(v)
            return v / n if n > 1e-9 else v
        d1a, d1c = unit(v1a), unit(v1c)
        d2a, d2c = unit(v2a), unit(v2c)
        if abs(d1a @ d2a) >= abs(d1a @ d2c):
            pair1_len_diff = abs(norm(v1a) - norm(v2a))
            pair1_cos = abs(d1a @ d2a)
            pair2_len_diff = abs(norm(v1c) - norm(v2c))
            pair2_cos = abs(d1c @ d2c)
        else:
            pair1_len_diff = abs(norm(v1a) - norm(v2c))
            pair1_cos = abs(d1a @ d2c)
            pair2_len_diff = abs(norm(v1c) - norm(v2a))
            pair2_cos = abs(d1c @ d2a)
        return (pair1_cos >= self.multi_edge_parallel_cos_min and
                pair2_cos >= self.multi_edge_parallel_cos_min and
                pair1_len_diff <= self.multi_edge_len_match_tol and
                pair2_len_diff <= self.multi_edge_len_match_tol)

    def cloud_callback(self, msg: PointCloud2):
        pts = []
        xyzrgb = []
        field_names = [f.name for f in msg.fields]
        if "rgb" in field_names or "rgba" in field_names:
            use = "rgba" if "rgba" in field_names else "rgb"
            for x,y,z,rgb in point_cloud2.read_points(msg, field_names=("x","y","z",use), skip_nans=True):
                r,g,b = unpack_rgb(rgb)
                s = r+g+b
                if s == 0: continue
                rn, gn = r/s, g/s
                if rn > 0.3508 and gn > 0.3508 and rn > gn:
                    pts.append((x,y))
                    xyzrgb.append((x,y,z))
        else:
            for x,y,z,r,g,b in point_cloud2.read_points(msg, field_names=("x","y","z","r","g","b"), skip_nans=True):
                s = r+g+b
                if s == 0: continue
                rn, gn = r/s, g/s
                if rn > 0.3508 and gn > 0.3508 and rn > gn:
                    pts.append((x,y))
                    xyzrgb.append((x,y,z))
        if len(pts) < self.min_yellow_points:
            self.get_logger().warn(f"Too few yellow points: {len(pts)} < {self.min_yellow_points}.")
            self.publish_line([])
            return
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
        lshape = self.detect_lshape(pts_np)
        if lshape:
            A, I, C = lshape
            len1 = np.linalg.norm(A - I)
            len2 = np.linalg.norm(C - I)
            if self.edge_accepts(len1, len2):
                coords = [f"{p[0]:.3f},{p[1]:.3f},{z_on_plane(p[0],p[1]):.3f}" for p in [A, I, C]]
                self.publish_edge(coords)
                self.publish_markers(msg.header.frame_id or "map", msg.header.stamp, [A, I, C], z_on_plane, xyzrgb)
                return
        line = self.detect_line(pts_np)
        if line:
            P1, P2 = line
            coords = [f"{P1[0]:.3f},{P1[1]:.3f},{z_on_plane(P1[0],P1[1]):.3f}",
                      f"{P2[0]:.3f},{P2[1]:.3f},{z_on_plane(P2[0],P2[1]):.3f}"]
            self.publish_line(coords)
            self.publish_markers(msg.header.frame_id or "map", msg.header.stamp, [P1, P2], z_on_plane, xyzrgb)

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

        # --------- Corner sanity checks (prevents fake/too-close corners) ---------
        in1 = np.array(in1)
        in2 = np.array(in2)
        L1 = np.linalg.norm(A - I)
        L2 = np.linalg.norm(C - I)

        # 1) Legs must be long enough
        if L1 < self.corner_min_leg or L2 < self.corner_min_leg:
            return None

        # 2) Clip inliers to each segment [I->A], [I->C] and enforce continuity
        s1 = (in1 - I) @ d1
        s2 = (in2 - I) @ d2
        seg1_mask = (s1 >= 0.0) & (s1 <= L1)
        seg2_mask = (s2 >= 0.0) & (s2 <= L2)
        s1_seg = s1[seg1_mask]
        s2_seg = s2[seg2_mask]

        def max_gap(s):
            if s.size < 2:
                return np.inf
            s_sorted = np.sort(s)
            return np.max(np.diff(s_sorted))

        if max_gap(s1_seg) > self.corner_max_gap or max_gap(s2_seg) > self.corner_max_gap:
            return None

        # 3) Balanced density across both legs
        dens1 = (seg1_mask.sum()) / max(L1, 1e-6)
        dens2 = (seg2_mask.sum()) / max(L2, 1e-6)
        ratio = max(dens1, dens2) / max(min(dens1, dens2), 1e-6)
        if ratio > self.edge_density_ratio_max:
            return None
        # --------------------------------------------------------------------------

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
