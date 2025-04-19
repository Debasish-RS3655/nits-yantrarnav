#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np

class FlatAreaProcessor(Node):
    def __init__(self):
        super().__init__('flat_area_processor')

        # Storage for phases and points
        self.phase = None
        self.ml_predicted_areas = []  # list of tuples: (np.array([x,y,z]), terrain)
        self.landing_spots = []       # list of np.array([x,y,z])
        self.flat_area_gap_threshold = 0.5

        # ROS 2 Subscriptions
        self.create_subscription(String, '/position/phase', self.phase_callback, 10)
        self.create_subscription(String, '/ml_predicted_area', self.ml_flat_area_callback, 10)
        self.create_subscription(Point, '/landing_spots', self.landing_spot_callback, 10)

        # ROS 2 Publisher: publishes combined flat areas at phase 3
        self.flat_area_pub = self.create_publisher(String, '/position/flat_area', 10)

    def phase_callback(self, msg: String):
        """Track phase; when entering phase 3, publish final flat areas."""
        try:
            new_phase = int(msg.data)
        except ValueError:
            self.get_logger().error(f"Invalid phase data received: {msg.data}")
            return

        if self.phase != 3 and new_phase == 3:
            # Transition into phase 3: compute and publish
            self.publish_final_flat_areas()
        self.phase = new_phase

    def ml_flat_area_callback(self, msg: String):
        """Accumulate ML-predicted points with terrain until phase 3."""
        try:
            # Expect: "class {terrain} coordinate x={x} y={y} z={z}"
            text = msg.data.strip()
            parts = text.split(' coordinate ')
            terrain = parts[0].replace('class ', '').strip().lower()

            coord_tokens = parts[1].split()
            coords = {k: float(v) for token in coord_tokens for k,v in [token.split('=')]}
            point = np.array([coords['x'], coords['y'], coords['z']], dtype=np.float64)

            # Store ml prediction
            self.ml_predicted_areas.append((point, terrain))
            self.get_logger().info(f"Stored ML-predicted area: {terrain} at {point.tolist()}")

        except Exception as e:
            self.get_logger().error(f"Error parsing ML message: {e}")

    def landing_spot_callback(self, msg: Point):
        """Accumulate unique landing spot points until phase 3."""
        new_point = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
        # Ensure uniqueness by distance
        if all(np.linalg.norm(new_point - p) >= self.flat_area_gap_threshold
               for p in self.landing_spots):
            self.landing_spots.append(new_point)
            self.get_logger().info(f"Added landing spot: {new_point.tolist()}")
        else:
            self.get_logger().info("Ignored duplicate landing spot.")

    def publish_final_flat_areas(self):
        """
        On entering phase 3, find ML-predicted areas that match landing spots,
        prioritize by terrain (hard > sandy > rocky), and publish.
        """
        # Terrain priority mapping
        priority = {'hard_terrain': 0, 'sandy_terrain': 1, 'rocky_terrain': 2}

        # Sort ml-predicted by priority
        sorted_ml = sorted(
            self.ml_predicted_areas,
            key=lambda item: priority.get(item[1], float('inf'))
        )

        # Filter: keep points that are near any landing spot
        final_points = []
        for point, terrain in sorted_ml:
            if any(np.linalg.norm(point - ls) < self.flat_area_gap_threshold
                   for ls in self.landing_spots):
                # avoid duplicates in final list
                if not any(np.allclose(point, fp) for fp in final_points):
                    final_points.append(point)

        # Format output as "x1=... y1=... z1=..., x2=... y2=... z2=..."
        parts = []
        for idx, pt in enumerate(final_points, start=1):
            x, y, z = int(pt[0]), int(pt[1]), int(pt[2])
            parts.append(f"x{idx}={x} y{idx}={y} z{idx}={z}")
        msg_str = ', '.join(parts) if parts else ''

        # Publish
        out_msg = String()
        out_msg.data = msg_str
        self.flat_area_pub.publish(out_msg)
        self.get_logger().info(f"Published flat areas: {msg_str}")


def main(args=None):
    rclpy.init(args=args)
    node = FlatAreaProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
