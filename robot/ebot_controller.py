import time
import math
import logging
from typing import Tuple, Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from .base import BaseRobotController

logger = logging.getLogger(__name__)

def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Compute yaw (rotation about Z) from quaternion."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a

class EbotRobotController(BaseRobotController, Node):
    """
    Direct ROS 2 controller for the ebot base.
    Uses proportional control on /cmd_vel based on /odom and /scan (Lidar).
    This controller is self-contained and does not require Nav2 or AMCL.
    """
    def __init__(self):
        # Initialize ROS 2 Node
        if not rclpy.ok():
            rclpy.init()
        Node.__init__(self, 'ebot_robot_controller')

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Internal pose state
        self.pose_x: Optional[float] = None
        self.pose_y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.last_scan: Optional[LaserScan] = None

        # Control Parameters (matches eYRC competition settings)
        self.max_ang = 1.0
        self.min_lin = 0.1
        self.k_lin = 1.2
        self.k_ang = 2.5
        self.avoid_front_thresh = 0.55
        self.stop_front_thresh = 0.30
        self.slow_front_thresh = 0.60
        self.pos_tol = 0.15
        self.yaw_tol = math.radians(50.0)

        # Spawning coordinates inside e-Yantra warehouse (default initial pose)
        # This aligns the robot's coordinates on startup
        logger.info("🤖 Ebot direct controller initialized. Listening to /odom and /scan...")

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.pose_x = pos.x
        self.pose_y = pos.y
        self.yaw = yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w)

    def _get_sector_min(self, start_angle: float, end_angle: float) -> float:
        """Return min range in sector [start_angle, end_angle] relative to front."""
        if self.last_scan is None:
            return 10.0

        scan = self.last_scan
        angle_min = scan.angle_min
        inc = scan.angle_increment
        n = len(scan.ranges)

        i0 = int(max(0, min(n - 1, round((start_angle - angle_min) / inc))))
        i1 = int(max(0, min(n - 1, round((end_angle - angle_min) / inc))))
        if i0 > i1:
            i0, i1 = i1, i0

        min_r = float('inf')
        for i in range(i0, i1 + 1):
            r = scan.ranges[i]
            if math.isfinite(r) and r > 0.0:
                if r < min_r:
                    min_r = r

        if not math.isfinite(min_r):
            return 10.0
        return min_r

    def navigate_to(self, target_name: str, x: float, y: float, theta: float, speed: float) -> bool:
        logger.info(f"🚀 [EbotRobot] Navigating directly to '{target_name}' ({x}, {y}) at {speed} m/s...")
        goal_reached = False
        start_time = time.time()
        max_duration = 120.0 # safety timeout per waypoint (2 minutes)

        while rclpy.ok() and not goal_reached:
            # Spin node to trigger subscriptions callbacks
            rclpy.spin_once(self, timeout_sec=0.05)

            # Wait for valid odometry reading
            if self.pose_x is None or self.yaw is None:
                continue

            # Check timeout limit
            if time.time() - start_time > max_duration:
                logger.error(f"❌ [EbotRobot] Navigation to '{target_name}' timed out! Aborting.")
                self.stop()
                return False

            dx = x - self.pose_x
            dy = y - self.pose_y
            dist = math.hypot(dx, dy)
            desired_heading = math.atan2(dy, dx)
            heading_err = normalize_angle(desired_heading - self.yaw)

            twist = Twist()

            if dist > self.pos_tol:
                # Proportional steering logic
                heading_threshold = math.radians(18)
                if abs(heading_err) > heading_threshold:
                    # Too far from correct heading - rotate in place
                    v_cmd = 0.0
                    w_cmd = self.k_ang * heading_err
                else:
                    # Move forward while correcting steering
                    forward_scale = max(0.0, math.cos(heading_err))
                    v_cmd = self.k_lin * dist * forward_scale
                    # Clamping target speed
                    v_cmd = max(self.min_lin, min(speed, v_cmd))
                    w_cmd = self.k_ang * heading_err

                # Limit angular velocity
                w_cmd = max(-self.max_ang, min(self.max_ang, w_cmd))

                # LiDAR obstacle avoidance check
                front_min = self._get_sector_min(math.radians(-30), math.radians(30))
                left_min = self._get_sector_min(math.radians(60), math.radians(120))
                right_min = self._get_sector_min(math.radians(-120), math.radians(-60))

                if front_min < self.stop_front_thresh:
                    # Obstacle too close -> stop and rotate away
                    twist.linear.x = 0.0
                    twist.angular.z = self.max_ang * (1.0 if left_min >= right_min else -1.0)
                elif front_min < self.avoid_front_thresh:
                    # Obstacle nearby -> slow down and steer
                    twist.linear.x = min(0.25, max(0.0, v_cmd * 0.4))
                    steer = 0.8 * self.max_ang * (1.0 if left_min >= right_min else -1.0)
                    twist.angular.z = max(-self.max_ang, min(self.max_ang, 0.4 * w_cmd + 0.6 * steer))
                else:
                    # Path clear -> standard speeds
                    if front_min < self.slow_front_thresh:
                        v_cmd *= 0.6
                    twist.linear.x = v_cmd
                    twist.angular.z = w_cmd

                self.cmd_pub.publish(twist)
            else:
                # Position reached - stop and mark goal as complete
                self.stop()
                goal_reached = True


        logger.info(f"✅ [EbotRobot] Arrived at '{target_name}'.")
        return True

    def stop(self) -> None:
        logger.warning("🛑 [EbotRobot] Stopping robot movement.")
        if rclpy.ok():
            try:
                twist = Twist()
                self.cmd_pub.publish(twist)
                # Give ROS 2 DDS middleware time to transmit the message before the process exits
                time.sleep(1.0)
            except Exception as e:
                logger.error(f"Failed to publish stop command: {e}")

