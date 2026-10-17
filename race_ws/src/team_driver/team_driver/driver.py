#!/usr/bin/env python3
import math
import os
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import qos_profile_sensor_data


class Driver(Node):

    def __init__(self):
        super().__init__('driver')

        # Load track centerline waypoints
        csv_path = '/hackathon/maps/icra26_centerline.csv'
        if os.path.exists(csv_path):
            self.waypoints = np.loadtxt(csv_path, delimiter=',')
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints from centerline.csv'
            )
        else:
            self.get_logger().error(
                f'Could not find waypoints file at {csv_path}'
            )
            self.waypoints = np.array([[0.0, 0.0]])

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')

        # Vehicle Physical Limits
        self.wheelbase = 0.33
        self.max_steering = 0.4189  # [rad]

        # SPEED & LOOKAHEAD BASELINE
        self.speed_max = 7.6  # [m/s] High-speed straightaway pace
        self.speed_min = 3.6  # [m/s] Hairpin speed
        self.lookahead_min = 0.98  # [m] Raised slightly to prevent sharp U-turn clipping
        self.lookahead_max = 2.08  # [m]

        self.position = None
        self.yaw = 0.0
        self.current_speed = 0.0
        self.front_clearance = 25.0

        # ROS 2 Subscriptions & Publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.get_parameter('drive_topic').value,
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/driver/markers', 1
        )

        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self._marker_divisor = 0

    def scan_callback(self, scan):
        """Monitor forward LiDAR rays for safety limits."""
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=25.0, neginf=0.0)

        num_beams = len(ranges)
        mid_idx = num_beams // 2
        window = int(num_beams * (30.0 / 270.0) / 2.0)

        front_arc = ranges[
            max(0, mid_idx - window) : min(num_beams, mid_idx + window)
        ]
        if len(front_arc) > 0:
            self.front_clearance = float(np.min(front_arc))

    def odom_callback(self, msg):
        self.position = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y]
        )
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.current_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y
        )

        steering, speed = self.plan(
            self.position[0], self.position[1], self.yaw
        )
        self.publish(steering, speed)

        self._marker_divisor = (self._marker_divisor + 1) % 5
        if self._marker_divisor == 0:
            self.publish_marker(steering)

    def plan(self, odom_x, odom_y, odom_yaw):
        if not hasattr(self, 'waypoints') or self.position is None:
            return 0.0, 0.0

        # 1. Find closest waypoint
        distances = np.linalg.norm(
            self.waypoints - np.array([odom_x, odom_y]), axis=1
        )
        closest_idx = int(np.argmin(distances))

        # 2. Curvature estimation (10-waypoint lookahead preview)
        preview_idx = (closest_idx + 10) % len(self.waypoints)
        far_idx = (closest_idx + 20) % len(self.waypoints)

        v_near = self.waypoints[preview_idx] - self.waypoints[closest_idx]
        v_far = self.waypoints[far_idx] - self.waypoints[preview_idx]

        angle_near = math.atan2(v_near[1], v_near[0])
        angle_far = math.atan2(v_far[1], v_far[0])
        heading_change = abs(
            math.atan2(
                math.sin(angle_far - angle_near),
                math.cos(angle_far - angle_near),
            )
        )

        # 3. Calculate target speed based on curvature
        curvature_factor = np.clip(heading_change / 0.75, 0.0, 1.0)
        target_speed = self.speed_max - curvature_factor * (
            self.speed_max - self.speed_min
        )

        # 4. Speed-adaptive lookahead calculation
        lookahead = np.clip(
            0.6 + 0.30 * self.current_speed,
            self.lookahead_min,
            self.lookahead_max,
        )

        # 5. Target waypoint selection
        target_idx = closest_idx
        for i in range(closest_idx + 1, closest_idx + 300):
            idx = i % len(self.waypoints)
            target_pt = self.waypoints[idx]

            dist = np.linalg.norm(target_pt - np.array([odom_x, odom_y]))
            dx = target_pt[0] - odom_x
            dy = target_pt[1] - odom_y
            local_x = math.cos(-odom_yaw) * dx - math.sin(-odom_yaw) * dy

            if dist >= lookahead and local_x > 0:
                target_idx = idx
                break

        if target_idx == closest_idx:
            target_idx = (closest_idx + 10) % len(self.waypoints)

        target_x, target_y = self.waypoints[target_idx]

        # 6. Transform target to local vehicle frame
        dx = target_x - odom_x
        dy = target_y - odom_y
        local_x = math.cos(-odom_yaw) * dx - math.sin(-odom_yaw) * dy
        local_y = math.sin(-odom_yaw) * dx + math.cos(-odom_yaw) * dy

        # High-Speed Sweeper Correction: Nudge target 8cm inward at high speeds on wide bends
        if self.current_speed > 6.2 and heading_change < 0.35:
            local_y -= np.sign(local_y) * 0.08

        # Smooth Pure Pursuit Steering Angle
        alpha = math.atan2(local_y, local_x)
        steering_angle = math.atan2(
            (2.0 * self.wheelbase * math.sin(alpha)), lookahead
        )
        steering_angle = float(
            np.clip(steering_angle, -self.max_steering, self.max_steering)
        )

        # 7. Safety Override
        if self.front_clearance < 1.4:
            lidar_speed_limit = max(0.5, self.front_clearance * 1.2)
            target_speed = min(target_speed, lidar_speed_limit)

        return steering_angle, target_speed

    def publish(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def publish_marker(self, target_angle):
        marker = Marker()
        marker.header.frame_id = 'ego_racecar/base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'team_driver'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = 1.0, 0.1, 0.1
        marker.color.g, marker.color.b, marker.color.a = 0.2, 1.0, 0.8
        marker.pose.orientation.z = math.sin(target_angle / 2.0)
        marker.pose.orientation.w = math.cos(target_angle / 2.0)
        array = MarkerArray()
        array.markers.append(marker)
        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()