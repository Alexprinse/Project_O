import yaml
from pathlib import Path
import logging
from .schema import MissionPlan

logger = logging.getLogger(__name__)

class MissionValidator:
    """
    Validates the structured mission plan against business and safety rules
    defined in the configuration YAML files.
    """
    def __init__(self, config_dir: str = "config", waypoints_file: str = "waypoints_turtlebot3.yaml"):
        self.config_dir = Path(config_dir)
        self.settings = self._load_yaml("settings.yaml")
        self.waypoints = self._load_yaml(waypoints_file)

    def _load_yaml(self, filename: str) -> dict:
        filepath = self.config_dir / filename
        if not filepath.exists():
            raise FileNotFoundError(f"Configuration file not found: {filepath}")
        with open(filepath, "r") as f:
            return yaml.safe_load(f)

    def validate(self, raw_mission: dict) -> MissionPlan:
        """
        Validates the raw dictionary against Pydantic schema and safety limits.
        Returns a validated MissionPlan object or raises ValueError.
        """
        # 1. Structural and type validation via Pydantic
        try:
            plan = MissionPlan(**raw_mission)
        except Exception as e:
            raise ValueError(f"Schema validation failed: {e}")

        # 2. Business logic & safety validation
        max_loops = self.settings.get("safety", {}).get("max_loops", 10)
        if plan.loops < 1 or plan.loops > max_loops:
            raise ValueError(f"Safety constraint violated: Loop count {plan.loops} is outside allowed range (1 - {max_loops})")

        if plan.speed is not None:
            min_speed = self.settings.get("safety", {}).get("min_speed", 0.1)
            max_speed = self.settings.get("safety", {}).get("max_speed", 1.5)
            if plan.speed < min_speed or plan.speed > max_speed:
                raise ValueError(f"Safety constraint violated: Speed {plan.speed}m/s is outside allowed bounds ({min_speed} - {max_speed}m/s)")
        else:
            plan.speed = self.settings.get("safety", {}).get("default_speed", 0.5)

        # 3. Route vs Custom Waypoints vs Target Object checks
        if plan.mission_type == "follow":
            if not plan.target_object:
                raise ValueError("Safety constraint violated: Mission type is 'follow' but no target_object is specified.")
        elif plan.mission_type == "split_patrol" and plan.agent_routes:
            # Explicit split patrol routes are defined per-agent, verify each route exists
            allowed_routes = self.waypoints.get("routes", {}).keys()
            for r in plan.agent_routes:
                if r.route not in allowed_routes:
                    raise ValueError(f"Safety constraint violated: Route '{r.route}' for agent '{r.agent_id}' is not a known route. Known routes: {list(allowed_routes)}")
        elif plan.mission_type in ("regroup", "regroup_home") and not plan.route and not plan.waypoints:
            # Regrouping to base station does not require a predefined route or waypoints
            pass
        else:
            if not plan.route and not plan.waypoints:
                raise ValueError("Safety constraint violated: Mission requires either a predefined 'route' or custom 'waypoints'.")

            if plan.route:
                allowed_routes = self.waypoints.get("routes", {}).keys()
                if plan.route not in allowed_routes:
                    raise ValueError(f"Safety constraint violated: Route '{plan.route}' is not a known route. Known routes: {list(allowed_routes)}")
            
            if plan.waypoints:
                x_min = self.settings.get("safety", {}).get("x_min", -10.0)
                x_max = self.settings.get("safety", {}).get("x_max", 10.0)
                y_min = self.settings.get("safety", {}).get("y_min", -12.0)
                y_max = self.settings.get("safety", {}).get("y_max", 12.0)
                
                for idx, wp in enumerate(plan.waypoints):
                    if not (x_min <= wp.x <= x_max):
                        raise ValueError(f"Safety constraint violated: Waypoint {idx} ({wp.name or 'unnamed'}) X coordinate ({wp.x}) is outside allowed bounds ({x_min} - {x_max})")
                    if not (y_min <= wp.y <= y_max):
                        raise ValueError(f"Safety constraint violated: Waypoint {idx} ({wp.name or 'unnamed'}) Y coordinate ({wp.y}) is outside allowed bounds ({y_min} - {y_max})")

        return plan

