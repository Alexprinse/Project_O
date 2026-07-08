import time
import math
import logging
from typing import List, Tuple, Dict, Optional
from concurrent.futures import ThreadPoolExecutor, wait
from robot.formation.formations import get_slot_offsets, project_slots_to_world, assign_agents_to_slots

logger = logging.getLogger(__name__)

class FormationManager:
    """
    Coordinates 2-3 robots in formations or split-patrol missions.
    Purely additive layer that runs on top of BaseRobotController.
    """
    def __init__(self, use_ros: bool, agent_ids: List[str]):
        self.use_ros = use_ros
        self.agent_ids = agent_ids
        self.agents = {}
        
        logger.info(f"Initializing FormationManager with agents: {agent_ids} (ROS mode: {use_ros})")
        
        for aid in self.agent_ids:
            if self.use_ros:
                from robot.ros2_controller import ROS2Nav2Controller
                self.agents[aid] = ROS2Nav2Controller(namespace=aid)
            else:
                from robot.mock_controller import MockRobotController
                self.agents[aid] = MockRobotController(namespace=aid)

    def get_agent_poses(self) -> Dict[str, Tuple[float, float, float]]:
        """
        Gets current (x, y, theta) poses of all agents.
        """
        poses = {}
        for aid, agent in self.agents.items():
            pose = agent._get_current_pose()
            # Fallback if odom hasn't received anything yet
            if pose[0] is None or pose[1] is None:
                x_init = -2.0
                y_init = -0.5
                if aid == "tb3_1":
                    y_init = 0.5
                elif aid == "tb3_2":
                    y_init = 1.5
                poses[aid] = (x_init, y_init, 0.0)
            else:
                poses[aid] = (pose[0], pose[1], pose[2] if pose[2] is not None else 0.0)
        return poses

    def stop_all(self):
        """
        Emergency stops all agents.
        """
        logger.warning("🛑 FormationManager: Stopping all squad agents!")
        for agent in self.agents.values():
            agent.stop()

    def set_agent_goal(self, agent_id: str, x: float, y: float, theta: float):
        """
        Sends a non-blocking goal to a specific namespaced robot.
        """
        agent = self.agents[agent_id]
        if self.use_ros:
            from geometry_msgs.msg import PoseStamped
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = agent.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.position.z = 0.0
            
            # Convert yaw theta to quaternion orientation
            goal_pose.pose.orientation.z = math.sin(theta / 2.0)
            goal_pose.pose.orientation.w = math.cos(theta / 2.0)
            
            agent.navigator.goToPose(goal_pose)
        else:
            # Mock mode: immediately update target position
            agent.current_x = x
            agent.current_y = y
            agent.current_theta = theta

    def form_up(self, formation_type: str, spacing: float) -> bool:
        """
        Assembles the squad into a formation at their current average center position.
        """
        logger.info(f"📐 Squad forming up into: {formation_type} (spacing: {spacing}m)...")
        agent_poses = self.get_agent_poses()
        
        # Calculate anchor point (average of all current positions)
        xs = [p[0] for p in agent_poses.values()]
        ys = [p[1] for p in agent_poses.values()]
        thetas = [p[2] for p in agent_poses.values()]
        
        anchor = (sum(xs) / len(xs), sum(ys) / len(ys))
        heading = sum(thetas) / len(thetas)
        
        # Calculate slot target world positions
        offsets = get_slot_offsets(formation_type, len(self.agent_ids), spacing)
        slots = project_slots_to_world(anchor, heading, offsets)
        
        # Assign closest agents to slots
        assignments = assign_agents_to_slots(
            {aid: (p[0], p[1]) for aid, p in agent_poses.items()},
            slots
        )
        
        # Drive all agents to their assigned slots in parallel
        logger.info("🚚 Driving agents to formation slots...")
        success = True
        with ThreadPoolExecutor(max_workers=len(self.agent_ids)) as executor:
            futures = []
            for aid, (sx, sy, stheta) in assignments.items():
                logger.info(f"   Agent '{aid}' heading to slot: ({sx:.2f}, {sy:.2f})")
                futures.append(
                    executor.submit(self.agents[aid].navigate_to, "slot", sx, sy, stheta, 0.4)
                )
            # Barrier wait
            results = [f.result() for f in futures]
            if not all(results):
                success = False
                
        if success:
            logger.info("✅ Squad successfully formed up!")
        else:
            logger.error("❌ Some agents failed to reach their formation slots.")
        return success

    def move_formation(self, formation_type: str, spacing: float, path: List[Tuple[float, float, float]], speed: float) -> bool:
        """
        Executes a trajectory in formation using Dynamic Offset Re-Planning (2 Hz).
        """
        if not self.agent_ids:
            return False
            
        leader_id = self.agent_ids[0]
        followers = self.agent_ids[1:]
        
        logger.info(f"🚀 Moving formation '{formation_type}' along {len(path)} waypoints at {speed} m/s...")
        
        for idx, wp in enumerate(path):
            wx, wy, wtheta = wp
            logger.info(f"📍 Waypoint {idx+1}/{len(path)}: ({wx}, {wy})")
            
            # Record start pose for mock interpolation
            agent_poses = self.get_agent_poses()
            start_x, start_y, start_theta = agent_poses[leader_id]
            
            if self.use_ros:
                # 1. Command leader to go to waypoint
                self.set_agent_goal(leader_id, wx, wy, wtheta)
                leader = self.agents[leader_id]
                
                # 2. Start 2 Hz followers coordination loop
                while not leader.navigator.isTaskComplete():
                    # Get leader's current position and heading
                    lx, ly, ltheta = leader._get_current_pose()
                    if lx is None or ly is None or ltheta is None:
                        time.sleep(0.5)
                        continue
                        
                    # Calculate follower slots relative to leader
                    offsets = get_slot_offsets(formation_type, len(self.agent_ids), spacing)
                    # Offsets start with leader at index 0, followers follow
                    follower_offsets = offsets[1:]
                    
                    projected_slots = project_slots_to_world((lx, ly), ltheta, follower_offsets)
                    
                    # Update each follower's active goal and spin its executor node
                    import rclpy
                    from robot.ros2_controller import _spin_lock
                    for f_idx, fid in enumerate(followers):
                        fx, fy, ftheta = projected_slots[f_idx]
                        self.set_agent_goal(fid, fx, fy, ftheta)
                        with _spin_lock:
                            rclpy.spin_once(self.agents[fid].navigator, timeout_sec=0.0)
                        
                    time.sleep(0.5)
                    
                # Check outcome of leader task
                from nav2_simple_commander.robot_navigator import TaskResult
                result = leader.navigator.getResult()
                if result != TaskResult.SUCCEEDED:
                    logger.error(f"❌ Leader failed to reach waypoint {idx+1}. Aborting formation.")
                    return False
            else:
                # Mock Mode: Simulate continuous motion
                dist = math.hypot(wx - start_x, wy - start_y)
                travel_time = dist / speed if speed > 0 else 1.0
                steps = max(2, int(travel_time / 0.5))
                
                for step in range(1, steps + 1):
                    ratio = step / steps
                    # Interpolated leader pose
                    lx = start_x + (wx - start_x) * ratio
                    ly = start_y + (wy - start_y) * ratio
                    ltheta = start_theta + (wtheta - start_theta) * ratio
                    
                    # Update leader pose
                    self.set_agent_goal(leader_id, lx, ly, ltheta)
                    
                    # Update follower offsets
                    offsets = get_slot_offsets(formation_type, len(self.agent_ids), spacing)
                    follower_offsets = offsets[1:]
                    projected_slots = project_slots_to_world((lx, ly), ltheta, follower_offsets)
                    
                    for f_idx, fid in enumerate(followers):
                        fx, fy, ftheta = projected_slots[f_idx]
                        self.set_agent_goal(fid, fx, fy, ftheta)
                        
                    time.sleep(0.1) # Accelerated sleep for mock snappiness
                    
        # Clean final alignment: navigate all agents to final strict formation positions
        agent_poses = self.get_agent_poses()
        lx, ly, ltheta = agent_poses[leader_id]
        offsets = get_slot_offsets(formation_type, len(self.agent_ids), spacing)
        final_slots = project_slots_to_world((lx, ly), ltheta, offsets)
        
        logger.info("🏁 Aligning all agents into final formation pose...")
        success = True
        with ThreadPoolExecutor(max_workers=len(self.agent_ids)) as executor:
            futures = []
            for f_idx, fid in enumerate(self.agent_ids):
                fx, fy, ftheta = final_slots[f_idx]
                futures.append(
                    executor.submit(self.agents[fid].navigate_to, "final_alignment", fx, fy, ftheta, speed)
                )
            results = [f.result() for f in futures]
            if not all(results):
                success = False
                
        return success
