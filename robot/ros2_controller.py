import time
import math
import logging
import threading
from typing import Optional, Any
from .base import BaseRobotController

logger = logging.getLogger(__name__)
_spin_lock = threading.Lock()

class ROS2Nav2Controller(BaseRobotController):
    """
    ROS 2 Controller using Nav2's BasicNavigator to command a robot (e.g., TurtleBot3) in Gazebo.
    """
    def __init__(self, namespace: str = "", wait_for_nav2: bool = True):
        import rclpy
        from nav2_simple_commander.robot_navigator import BasicNavigator
        
        self.namespace = namespace
        
        # Initialize ROS 2 rclpy if not already initialized
        if not rclpy.ok():
            rclpy.init()
            
        self.navigator = BasicNavigator(namespace=self.namespace) if self.namespace else BasicNavigator()
        
        # Odometry tracking for initial pose detection and intermediate waypoint fallback
        self._current_x = None
        self._current_y = None
        self._current_theta = None
        
        from nav_msgs.msg import Odometry
        def odom_callback(msg):
            self._current_x = msg.pose.pose.position.x
            self._current_y = msg.pose.pose.position.y
            
            # Compute yaw from quaternion orientation
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self._current_theta = math.atan2(siny_cosp, cosy_cosp)
            
        odom_topic = f"/{self.namespace}/odom" if self.namespace else "/odom"
        self._odom_sub = self.navigator.create_subscription(
            Odometry, 
            odom_topic, 
            odom_callback, 
            10
        )
        
        # Wait up to 5 seconds for the first odometry message to auto-detect starting pose
        logger.info(f"⏳ [{self.namespace if self.namespace else 'root'}] Auto-detecting spawn coordinates from odometry...")
        for _ in range(50):
            if self._current_x is not None:
                break
            with _spin_lock:
                rclpy.spin_once(self.navigator, timeout_sec=0.1)
                
        if self._current_x is not None:
            x_init = self._current_x
            y_init = self._current_y
            theta_init = self._current_theta
            logger.info(f"✨ Auto-detected spawn pose for '{self.namespace}': ({x_init:.2f}, {y_init:.2f}, {theta_init:.2f})")
        else:
            # Fallback to defaults if no odometry is received
            x_init = -2.0
            y_init = -0.5
            if self.namespace == "tb3_1":
                y_init = 0.5
            elif self.namespace == "tb3_2":
                y_init = -1.5
            theta_init = 0.0
            logger.warning(f"⚠️ Failed to detect spawn pose. Using fallback default: ({x_init}, {y_init})")
            
        from geometry_msgs.msg import PoseStamped
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = x_init
        initial_pose.pose.position.y = y_init
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.z = math.sin(theta_init / 2.0)
        initial_pose.pose.orientation.w = math.cos(theta_init / 2.0)
        self.navigator.setInitialPose(initial_pose)
        
        if wait_for_nav2:
            # Wait up to 5 seconds for DDS service discovery to find either localizer
            localizer_service = None
            logger.info("🔍 Auto-detecting active localization service...")
            for _ in range(10):
                services = [s[0] for s in self.navigator.get_service_names_and_types()]
                if any("slam_toolbox" in s for s in services):
                    localizer_service = "planner_server" # Use planner_server to bypass slam_toolbox lifecycle checks
                    logger.info("ℹ️ Detected SLAM Toolbox. Waiting on planner_server lifecycle activation.")
                    break
                elif any(s.endswith("amcl/get_state") for s in services):
                    localizer_service = "amcl"
                    logger.info("ℹ️ Detected AMCL as active localizer.")
                    break
                time.sleep(0.5)

            if localizer_service is None:
                logger.warning("⚠️ No active localizer service discovered yet. Defaulting to 'planner_server'.")
                localizer_service = "planner_server"

            # Wait for Nav2 to be fully active
            logger.info("⏳ Waiting for Nav2 to become active...")
            with _spin_lock:
                self.navigator.waitUntilNav2Active(localizer=localizer_service)
            logger.info("✅ Nav2 is active and ready.")

    def _get_current_pose(self) -> tuple[Optional[float], Optional[float], Optional[float]]:
        return self._current_x, self._current_y, self._current_theta

    def navigate_to(self, target_name: str, x: float, y: float, theta: float, speed: float, depth: int = 0) -> bool:
        from geometry_msgs.msg import PoseStamped
        from nav2_simple_commander.robot_navigator import TaskResult
        
        if depth > 10:
            logger.error("❌ Max recursion depth reached for intermediate navigation. Aborting.")
            return False
            
        if depth == 0:
            logger.info(f"🚀 [ROS2Nav2] Sending goal '{target_name}' at ({x}, {y}) to Nav2...")
        else:
            logger.info(f"🚧 [ROS2Nav2] Sending intermediate goal at ({x:.2f}, {y:.2f}) (depth {depth})...")
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        
        # Convert theta (yaw) to quaternion
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Note: Actual motor speed is handled by Nav2's costmaps and local trajectory planner.
        logger.info(f"   [ROS2Nav2] AI requested speed: {speed} m/s.")

        with _spin_lock:
            self.navigator.goToPose(goal_pose)
        
        i = 0
        while True:
            with _spin_lock:
                is_complete = self.navigator.isTaskComplete()
            if is_complete:
                break
            i += 1
            with _spin_lock:
                feedback = self.navigator.getFeedback()
            # Print feedback every few iterations so we don't spam the console
            if feedback and i % 5 == 0:
                logger.info(f"   [ROS2Nav2] Distance remaining: {feedback.distance_remaining:.2f} meters")
            time.sleep(0.5)
            
        with _spin_lock:
            result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if depth == 0:
                logger.info(f"✅ [ROS2Nav2] Successfully arrived at '{target_name}'.")
            else:
                logger.info(f"✅ [ROS2Nav2] Successfully arrived at intermediate waypoint.")
            return True
        elif result == TaskResult.CANCELED:
            logger.warning(f"⚠️ [ROS2Nav2] Goal to '{target_name}' was canceled.")
            return False
        elif result == TaskResult.FAILED:
            logger.warning(f"⚠️ [ROS2Nav2] Failed to reach '{target_name}'. Checking intermediate waypoints...")
            
            # Fetch current pose
            curr_x, curr_y, _ = self._get_current_pose()
            if curr_x is None:
                logger.error("❌ Could not determine current pose. Aborting.")
                return False
                
            dx = x - curr_x
            dy = y - curr_y
            dist = math.sqrt(dx**2 + dy**2)
            
            # Segment only if the goal is far enough (e.g. > 2.0 meters) to warrant intermediate points
            if dist > 2.0:
                step = 2.0
                ux = dx / dist
                uy = dy / dist
                inter_x = curr_x + ux * step
                inter_y = curr_y + uy * step
                
                logger.info(f"🚧 Segmenting path: calculating intermediate point {step:.1f}m away towards final goal...")
                
                # Navigate to intermediate pose
                success = self.navigate_to(f"{target_name}_inter", inter_x, inter_y, theta, speed, depth + 1)
                if success:
                    logger.info("✅ Intermediate waypoint reached. Resuming to final goal...")
                    # Retry original goal
                    return self.navigate_to(target_name, x, y, theta, speed, depth + 1)
                else:
                    logger.error(f"❌ Failed to reach intermediate waypoint at ({inter_x:.2f}, {inter_y:.2f}).")
                    return False
            else:
                logger.error(f"❌ Target is too close ({dist:.2f}m) to segment further. Aborting.")
                return False
        return False
        
    def stop(self) -> None:
        logger.warning("🛑 [ROS2Nav2] Emergency Stop triggered! Canceling Nav2 goals.")
        with _spin_lock:
            self.navigator.cancelTask()
        
        # Publish empty twist to ensure robot stops moving
        try:
            from geometry_msgs.msg import Twist
            if not hasattr(self, "cmd_pub"):
                cmd_vel_topic = f"/{self.namespace}/cmd_vel" if self.namespace else "/cmd_vel"
                self.cmd_pub = self.navigator.create_publisher(Twist, cmd_vel_topic, 10)
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def follow_target(self, target_name: str, speed: Optional[float]) -> bool:
        import rclpy
        from sensor_msgs.msg import Image
        from geometry_msgs.msg import Twist
        import cv2
        import numpy as np
        from ultralytics import YOLO
        
        # Ensure any active Nav2 tasks are cancelled
        with _spin_lock:
            self.navigator.cancelTask()
        
        # Create publisher if not exists
        if not hasattr(self, "cmd_pub"):
            cmd_vel_topic = f"/{self.namespace}/cmd_vel" if self.namespace else "/cmd_vel"
            self.cmd_pub = self.navigator.create_publisher(Twist, cmd_vel_topic, 10)
        if not hasattr(self, "image_pub"):
            image_annotated_topic = f"/{self.namespace}/camera/image_annotated" if self.namespace else "/camera/image_annotated"
            self.image_pub = self.navigator.create_publisher(Image, image_annotated_topic, 10)
            
        # Shared image state
        self._latest_image = None
        def img_callback(msg):
            self._latest_image = msg
            
        image_raw_topic = f"/{self.namespace}/camera/image_raw" if self.namespace else "/camera/image_raw"
        sub = self.navigator.create_subscription(Image, image_raw_topic, img_callback, 10)
        
        logger.info(f"👁️ Starting visual follow node for target '{target_name}' using YOLOv8...")
        
        # Proportional controller gains
        k_ang = 0.003 # steer based on offset from center
        k_lin = 0.5   # forward velocity
        
        target_speed = speed if speed else 0.4
        
        # Initialize YOLOv8 nano model
        try:
            model = YOLO("yolov8n.pt")
        except Exception as e:
            logger.error(f"Failed to load YOLOv8 model: {e}")
            return False
            
        image_saved = False
        start_time = time.time()
        timeout = 180.0 # 3 minutes timeout
        
        lost_counter = 0
        
        try:
            while rclpy.ok():
                # Spin once to trigger subscription
                with _spin_lock:
                    rclpy.spin_once(self.navigator, timeout_sec=0.05)
                
                if time.time() - start_time > timeout:
                    logger.warning("⏱️ Target follow timed out!")
                    break
                    
                if self._latest_image is None:
                    continue
                    
                # Convert image manually from ROS message bytes to CV2 image
                msg = self._latest_image
                height, width = msg.height, msg.width
                
                try:
                    if msg.encoding == 'rgb8':
                        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    elif msg.encoding == 'bgr8':
                        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                    else:
                        # Fallback for other encodings
                        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                except Exception as e:
                    logger.error(f"Image conversion failed: {e}")
                    continue
                
                # Run YOLOv8 target detection
                target_box = self._detect_yolo(img, model, target_name.lower())
                    
                twist = Twist()
                
                if target_box is not None:
                    lost_counter = 0
                    tx, ty, tw, th = target_box
                    
                    # Draw detection bounding box
                    cv2.rectangle(img, (tx, ty), (tx+tw, ty+th), (0, 255, 0), 2)
                    cv2.putText(img, f"TARGET: {target_name}", (tx, ty-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Save snapshot
                    if not image_saved:
                        cv2.imwrite("detection.jpg", img)
                        logger.info("🎯 Target acquired! Snapshot saved to 'detection.jpg'")
                        image_saved = True
                        
                    # Calculate center offset
                    target_center_x = tx + tw / 2.0
                    image_center_x = width / 2.0
                    offset_x = image_center_x - target_center_x # positive if target is to the left
                    
                    # Angular velocity: proportional to offset
                    twist.angular.z = k_ang * offset_x
                    # Clamp angular velocity
                    twist.angular.z = max(-0.4, min(0.4, twist.angular.z))
                    
                    # Linear velocity: proportional to target size (closer = larger box area)
                    width_ratio = tw / float(width)
                    if width_ratio > 0.25:
                        logger.info("🏁 Arrived near the target. Stopping.")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.cmd_pub.publish(twist)
                        break
                    else:
                        twist.linear.x = target_speed * (1.0 - (width_ratio / 0.45))
                        twist.linear.x = max(0.05, min(target_speed, twist.linear.x))
                        
                    logger.info(f"   [VisionFollow] Tracking: offset={offset_x:.1f}px, ratio={width_ratio:.2f}, cmd_vel=({twist.linear.x:.2f}, {twist.angular.z:.2f})")
                else:
                    lost_counter += 1
                    if lost_counter > 20: # Lost for ~1 second
                        # Spin slowly to scan
                        logger.warning(f"🔍 Target '{target_name}' lost! Scanning...")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.3
                    else:
                        # Keep previous velocity or stop
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        
                # Publish the annotated camera frame to ROS 2 topic /camera/image_annotated
                try:
                    annotated_msg = Image()
                    annotated_msg.header.stamp = self.navigator.get_clock().now().to_msg()
                    annotated_msg.header.frame_id = 'camera_link'
                    annotated_msg.height = height
                    annotated_msg.width = width
                    annotated_msg.encoding = 'bgr8'
                    annotated_msg.is_bigendian = 0
                    annotated_msg.step = width * 3
                    annotated_msg.data = img.tobytes()
                    self.image_pub.publish(annotated_msg)
                except Exception as e:
                    logger.debug(f"Failed to publish annotated image: {e}")
                    
                self.cmd_pub.publish(twist)
                time.sleep(0.05)
                
        finally:
            # Stop the robot and clean up subscription
            stop_twist = Twist()
            self.cmd_pub.publish(stop_twist)
            self.navigator.destroy_subscription(sub)
            
        return image_saved

    def _detect_yolo(self, img, model, target_class: str) -> Optional[tuple[int, int, int, int]]:
        results = model.predict(source=img, verbose=False)
        best_box = None
        max_conf = 0.0
        detections = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0].item())
                conf = box.conf[0].item()
                label = model.names[cls_id].lower()
                
                if conf > 0.15:
                    detections.append(f"{label} ({conf:.2f})")
                
                # Alias mapping: Gazebo can model 'beer' is often classified as 'cup' or 'bottle'
                is_match = False
                if target_class == "bottle" and label in ["bottle", "cup"]:
                    is_match = True
                elif target_class == "cup" and label in ["bottle", "cup"]:
                    is_match = True
                elif target_class == "tree" and label in ["potted plant"]:
                    is_match = True
                elif target_class == "car" and label in ["car", "truck", "bus"]:
                    is_match = True
                elif label == target_class:
                    is_match = True
                    
                if is_match and conf > 0.25:
                    if conf > max_conf:
                        max_conf = conf
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        best_box = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))
                        
        if len(detections) > 0 and best_box is None:
            logger.info(f"   [YOLO Debug] Saw: {', '.join(detections[:3])} (Target: {target_class})")
            
        return best_box

        
    def __del__(self):
        # Do not call rclpy.shutdown() here, as it shuts down the ROS 2 context globally
        # for all other active nodes/controllers in the same process.
        pass
