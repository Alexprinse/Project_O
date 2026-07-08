import time
import math
import logging
from .base import BaseRobotController

logger = logging.getLogger(__name__)

class MockRobotController(BaseRobotController):
    """
    A mock simulator for testing the mission pipeline natively on macOS 
    without needing ROS 2 or Gazebo installed.
    It simulates travel time based on distance and speed, and prints telemetry.
    """
    def __init__(self, namespace: str = ""):
        self.namespace = namespace
        # Spaced starting positions to simulate multiple robots in mock mode
        self.current_x = -2.0
        self.current_y = -0.5
        if self.namespace == "tb3_1":
            self.current_y = 0.5
        elif self.namespace == "tb3_2":
            self.current_y = 1.5
        self.current_theta = 0.0
        
    def _get_current_pose(self) -> tuple[float, float, float]:
        return self.current_x, self.current_y, self.current_theta
        
    def navigate_to(self, target_name: str, x: float, y: float, theta: float, speed: float) -> bool:
        logger.info(f"🚀 [MockRobot] Navigating to '{target_name}' ({x}, {y}) at {speed} m/s...")
        
        distance = math.sqrt((x - self.current_x)**2 + (y - self.current_y)**2)
        if distance > 0 and speed > 0:
            travel_time = distance / speed
            logger.info(f"   [MockRobot] Distance: {distance:.2f}m. Estimated travel time: {travel_time:.2f}s")
            
            # Simulate travel with small sleeps to show progress
            steps = 5
            sleep_per_step = travel_time / steps
            
            # Cap the sleep per step in mock mode to keep tests snappy
            if sleep_per_step > 0.5:
                sleep_per_step = 0.5 
                
            for i in range(1, steps + 1):
                time.sleep(sleep_per_step)
                logger.info(f"   [MockRobot] Progress: {int((i/steps)*100)}%...")
                
        self.current_x = x
        self.current_y = y
        self.current_theta = theta
        
        logger.info(f"✅ [MockRobot] Arrived at '{target_name}'.")
        return True
        
    def stop(self) -> None:
        logger.warning("🛑 [MockRobot] Emergency Stop triggered!")

    def follow_target(self, target_name: str, speed: float) -> bool:
        logger.info(f"🚀 [MockRobot] Initializing search loop for target '{target_name}'...")
        time.sleep(1.0)
        logger.info(f"🔍 [MockRobot] Scanning environment with camera...")
        time.sleep(1.5)
        logger.info(f"🎯 [MockRobot] Target '{target_name}' acquired in camera frame!")
        time.sleep(1.0)
        logger.info(f"📸 [MockRobot] Sending picture to operator. Saving to project root as 'detection.jpg'")
        
        # Save a mock image to project root
        try:
            import numpy as np
            import cv2
            # Create a 300x300 image with text
            img = np.zeros((300, 300, 3), dtype=np.uint8)
            cv2.putText(img, f"MOCK: {target_name}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imwrite("detection.jpg", img)
            logger.info("✅ Saved mock image to detection.jpg")
        except Exception as e:
            logger.warning(f"Could not save mock image: {e}")
            
        logger.info(f"🔄 [MockRobot] Following target at distance: 2.0 meters at {speed} m/s...")
        for i in range(1, 4):
            time.sleep(0.8)
            logger.info(f"   [MockRobot] Tracking target: center offset = {5*i}px, distance = {2.0 - 0.2*i}m")
        logger.info(f"✅ [MockRobot] Successfully followed target '{target_name}'.")
        return True

