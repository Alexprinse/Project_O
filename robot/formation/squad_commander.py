import logging
from typing import List, Tuple, Dict, Optional
from robot.formation.manager import FormationManager
from robot.formation.tasks import execute_split_patrol, execute_explicit_patrol, execute_regroup

logger = logging.getLogger(__name__)

class SquadCommander:
    """
    High-level commander interface. Translates validated JSON MissionPlans
    into formation movements, split patrols, or regroup tasks on the fleet.
    """
    def __init__(self, use_ros: bool, agent_ids: List[str] = None):
        self.use_ros = use_ros
        # Default to 3-agent squad if not specified
        self.agent_ids = agent_ids if agent_ids else ["tb3_0", "tb3_1", "tb3_2"]
        self.manager = FormationManager(use_ros=self.use_ros, agent_ids=self.agent_ids)

    def execute_squad_mission(self, plan, waypoints_config: dict) -> bool:
        """
        Executes a validated mission plan on the squad.
        plan: validator.schema.MissionPlan
        waypoints_config: loaded waypoints YAML dict
        """
        mission_type = plan.mission_type.lower()
        speed = plan.speed if plan.speed is not None else 0.4
        
        logger.info(f"SquadCommander: Triggering squad mission '{mission_type}' at speed {speed} m/s...")
        
        if mission_type == "formation":
            formation_type = plan.formation_type if plan.formation_type else "wedge"
            spacing = plan.spacing if plan.spacing is not None else 1.0
            
            # Load path waypoints
            path = []
            if plan.route:
                if "routes" in waypoints_config and plan.route in waypoints_config["routes"]:
                    raw_wps = waypoints_config["routes"][plan.route]
                    path = [(wp["x"], wp["y"], wp.get("theta", 0.0)) for wp in raw_wps]
                else:
                    logger.error(f"Route '{plan.route}' not found in waypoint configuration.")
                    return False
            elif plan.waypoints:
                path = [(wp.x, wp.y, wp.theta) for wp in plan.waypoints]
            else:
                logger.error("No path or route waypoints defined for formation mission.")
                return False
                
            # Step 1: Form up at current position
            if not self.manager.form_up(formation_type, spacing):
                logger.error("Failed to assemble formation. Aborting mission.")
                return False
                
            # Step 2: Navigate along path in formation
            success = self.manager.move_formation(formation_type, spacing, path, speed)
            
            # Step 3: Optional return home
            if success and plan.return_home:
                logger.info("🏡 Squad return home requested. Navigating squad back to base...")
                home_pose = waypoints_config.get("home", {"x": -2.0, "y": -0.5, "theta": 0.0})
                success = execute_regroup(self.manager, (home_pose["x"], home_pose["y"]), speed)
                
            return success
            
        elif mission_type == "split_patrol":
            success = False
            # Check if explicit route assignments are defined
            if plan.agent_routes:
                success = execute_explicit_patrol(self.manager, [r.dict() for r in plan.agent_routes], waypoints_config, speed)
            # Check if implicit route splitting is defined
            elif plan.route:
                if "routes" in waypoints_config and plan.route in waypoints_config["routes"]:
                    raw_wps = waypoints_config["routes"][plan.route]
                    path = [(wp["x"], wp["y"], wp.get("theta", 0.0)) for wp in raw_wps]
                    success = execute_split_patrol(self.manager, path, speed)
                else:
                    logger.error(f"Route '{plan.route}' not found in waypoint configuration.")
                    return False
            else:
                logger.error("No route or agent routes defined for split patrol.")
                return False
                
            # Optional return home (regroup at home)
            if success and plan.return_home:
                logger.info("🏡 Squad return home requested. Regrouping squad at base...")
                home_pose = waypoints_config.get("home", {"x": -2.0, "y": -0.5, "theta": 0.0})
                success = execute_regroup(self.manager, (home_pose["x"], home_pose["y"]), speed)
                
            return success
            
        elif mission_type == "regroup" or "regroup" in plan.mission_type:
            # Regroup at base/home or first waypoint
            target = (0.0, 0.0)
            if plan.waypoints:
                target = (plan.waypoints[0].x, plan.waypoints[0].y)
            else:
                home_pose = waypoints_config.get("home", {"x": -2.0, "y": -0.5})
                target = (home_pose["x"], home_pose["y"])
                
            return execute_regroup(self.manager, target, speed)
            
        else:
            logger.error(f"Unsupported squad mission type: {mission_type}")
            return False
