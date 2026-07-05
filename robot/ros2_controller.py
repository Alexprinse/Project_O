import time
import math
import logging
from .base import BaseRobotController

logger = logging.getLogger(__name__)

class ROS2Nav2Controller(BaseRobotController):
    """
    ROS 2 Controller using Nav2's BasicNavigator to command a robot (e.g., TurtleBot3) in Gazebo.
    """
    def __init__(self):
        import rclpy
        from nav2_simple_commander.robot_navigator import BasicNavigator
        
        # Initialize ROS 2 rclpy if not already initialized
        if not rclpy.ok():
            rclpy.init()
            
        self.navigator = BasicNavigator()
        
        # Set initial pose (TurtleBot3 world default starting pose is x=-2.0, y=-0.5)
        from geometry_msgs.msg import PoseStamped
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -2.0
        initial_pose.pose.position.y = -0.5
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)
        
        # Wait up to 5 seconds for DDS service discovery to find either localizer
        localizer_service = None
        logger.info("🔍 Auto-detecting active localization service...")
        for _ in range(10):
            services = [s[0] for s in self.navigator.get_service_names_and_types()]
            if "/slam_toolbox/get_state" in services:
                localizer_service = "planner_server" # Use planner_server to bypass slam_toolbox lifecycle checks
                logger.info("ℹ️ Detected SLAM Toolbox. Waiting on planner_server lifecycle activation.")
                break
            elif "/amcl/get_state" in services:
                localizer_service = "amcl"
                logger.info("ℹ️ Detected AMCL as active localizer.")
                break
            time.sleep(0.5)

        if localizer_service is None:
            logger.warning("⚠️ No active localizer service discovered yet. Defaulting to 'planner_server'.")
            localizer_service = "planner_server"

        # Wait for Nav2 to be fully active
        logger.info("⏳ Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active(localizer=localizer_service)
        logger.info("✅ Nav2 is active and ready.")

    def navigate_to(self, target_name: str, x: float, y: float, theta: float, speed: float) -> bool:
        from geometry_msgs.msg import PoseStamped
        from nav2_simple_commander.robot_navigator import TaskResult
        
        logger.info(f"🚀 [ROS2Nav2] Sending goal '{target_name}' at ({x}, {y}) to Nav2...")
        
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

        self.navigator.goToPose(goal_pose)
        
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            # Print feedback every few iterations so we don't spam the console
            if feedback and i % 5 == 0:
                logger.info(f"   [ROS2Nav2] Distance remaining: {feedback.distance_remaining:.2f} meters")
            time.sleep(0.5)
            
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            logger.info(f"✅ [ROS2Nav2] Successfully arrived at '{target_name}'.")
            return True
        elif result == TaskResult.CANCELED:
            logger.warning(f"⚠️ [ROS2Nav2] Goal to '{target_name}' was canceled.")
            return False
        elif result == TaskResult.FAILED:
            logger.error(f"❌ [ROS2Nav2] Failed to reach '{target_name}'.")
            return False
        return False
        
    def stop(self) -> None:
        logger.warning("🛑 [ROS2Nav2] Emergency Stop triggered! Canceling Nav2 goals.")
        self.navigator.cancelTask()
        
    def __del__(self):
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
