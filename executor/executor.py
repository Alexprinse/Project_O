import logging
from validator.schema import MissionPlan
from robot.base import BaseRobotController

logger = logging.getLogger(__name__)

class MissionExecutor:
    """
    Deterministic executor that translates a validated MissionPlan into 
    concrete commands for a RobotController.
    This layer contains no AI reasoning. The same JSON always produces the same behavior.
    """
    def __init__(self, robot: BaseRobotController, waypoints_config: dict):
        self.robot = robot
        self.waypoints = waypoints_config
        
    def execute(self, plan: MissionPlan) -> None:
        logger.info("=" * 50)
        logger.info(f"🚦 STARTING MISSION EXECUTION")
        logger.info(f"Mission Type: {plan.mission_type}")
        logger.info(f"Route: {plan.route}")
        logger.info(f"Loops: {plan.loops}")
        logger.info(f"Speed: {plan.speed} m/s")
        logger.info("=" * 50)
        
        # Intercept squad/multi-agent missions
        squad_missions = ["formation", "split_patrol", "regroup"]
        is_squad = plan.mission_type in squad_missions or plan.agents is not None or plan.agent_routes is not None
        
        if is_squad:
            logger.info("👥 Detected multi-agent / squad mission. Delegating to SquadCommander...")
            
            # Determine ROS mode based on class type
            try:
                from robot.ros2_controller import ROS2Nav2Controller
                use_ros = isinstance(self.robot, ROS2Nav2Controller)
            except ImportError:
                use_ros = False
                
            # Determine agent list (extract from agent_routes if not explicitly given)
            agent_ids = plan.agents
            if not agent_ids and plan.agent_routes:
                agent_ids = list(dict.fromkeys(r.agent_id for r in plan.agent_routes))
                
            from robot.formation.squad_commander import SquadCommander
            commander = SquadCommander(use_ros=use_ros, agent_ids=agent_ids)
            
            success = commander.execute_squad_mission(plan, self.waypoints)
            if not success:
                logger.error("❌ Squad mission failed.")
                commander.manager.stop_all()
            logger.info("=" * 50)
            logger.info(f"🏁 SQUAD MISSION COMPLETE")
            logger.info("=" * 50)
            return

        # 1. Waypoint Navigation
        route_waypoints = []
        if plan.route:
            route_waypoints = self.waypoints.get("routes", {}).get(plan.route, [])
            if not route_waypoints:
                logger.error(f"Route '{plan.route}' has no waypoints defined!")
                return
        elif plan.waypoints:
            route_waypoints = [
                {"name": wp.name or f"wp_{i}", "x": wp.x, "y": wp.y, "theta": wp.theta}
                for i, wp in enumerate(plan.waypoints)
            ]
            
        if route_waypoints:
            for loop_idx in range(1, plan.loops + 1):
                logger.info(f"🔄 Starting Loop {loop_idx}/{plan.loops}...")
                for wp in route_waypoints:
                    name = wp.get("name", "unknown")
                    x = wp.get("x", 0.0)
                    y = wp.get("y", 0.0)
                    theta = wp.get("theta", 0.0)
                    
                    success = self.robot.navigate_to(name, x, y, theta, plan.speed)
                    if not success:
                        logger.error(f"❌ Failed to navigate to {name}. Aborting mission.")
                        self.robot.stop()
                        return
                logger.info(f"✅ Loop {loop_idx} completed.")
                
        # 2. Target Follow (Challenge 3)
        if plan.target_object or plan.mission_type == "follow":
            target = plan.target_object or "red"  # fallback if target is implicit
            if hasattr(self.robot, "follow_target"):
                logger.info(f"👁️ Starting target follow behavior for: '{target}'")
                success = self.robot.follow_target(target, plan.speed)
                if not success:
                    logger.error("❌ Target tracking failed or timed out.")
            else:
                logger.error("Robot controller does not support target tracking/following!")
                return
                
        # 3. Return Home (Only if not tracking or tracking finished)
        if plan.return_home and not plan.target_object:
            logger.info("🏠 Returning home...")
            home_wp = self.waypoints.get("home", {"x": 0.0, "y": 0.0, "theta": 0.0})
            self.robot.navigate_to("home", home_wp["x"], home_wp["y"], home_wp["theta"], plan.speed)
            
        logger.info("=" * 50)
        logger.info(f"🏁 MISSION COMPLETE")
        logger.info("=" * 50)
