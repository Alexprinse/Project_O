import argparse
import logging
import json
import sys

from llm.client import MissionParser
from validator.validator import MissionValidator
from robot.mock_controller import MockRobotController
from executor.executor import MissionExecutor

# Setup clean logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("Omokai")

def load_env():
    import os
    from pathlib import Path
    env_path = Path(".env")
    if env_path.exists():
        with open(env_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" in line:
                    key, val = line.split("=", 1)
                    key = key.strip()
                    val = val.strip().strip("'\"")
                    os.environ[key] = val

def main():
    load_env()
    parser = argparse.ArgumentParser(description="Omokai Robotics: Natural Language Mission Pipeline")
    parser.add_argument("--prompt", type=str, required=True, help="Natural language prompt for the robot")
    parser.add_argument("--mock-llm", action="store_true", help="Force using the mock LLM parser instead of Gemini API")
    parser.add_argument("--ros", action="store_true", help="Use ROS 2 Controller instead of Mock (requires Ubuntu/ROS2)")
    parser.add_argument("--robot", type=str, choices=["turtlebot3", "ebot"], default="turtlebot3", help="Robot platform configuration")
    args = parser.parse_args()
    
    logger.info("🤖 Starting Omokai Robotics Mission Pipeline...")
    
    # 1. Initialize Parser and Validator
    try:
        waypoints_file = f"waypoints_{args.robot}.yaml"
        validator = MissionValidator(config_dir="config", waypoints_file=waypoints_file)
        
        # Extract unique known waypoints with coordinates to pass to parser
        known_waypoints = {}
        routes_dict = validator.waypoints.get("routes", {})
        for route_name, wps in routes_dict.items():
            for wp in wps:
                if "name" in wp:
                    known_waypoints[wp["name"]] = {
                        "x": wp["x"],
                        "y": wp["y"],
                        "theta": wp.get("theta", 0.0)
                    }
        
        # If settings say use_mock is true, override the CLI flag
        use_mock = args.mock_llm or validator.settings.get("llm", {}).get("use_mock", False)
        routes = list(validator.waypoints.get("routes", {}).keys())
        llm_parser = MissionParser(use_mock=use_mock, allowed_routes=routes, known_waypoints=known_waypoints)
    except Exception as e:
        logger.error(f"Failed to initialize system validation: {e}")
        sys.exit(1)

    # 2. Parse Prompt (Prompt -> Raw JSON dict)
    logger.info(f"🗣️ User Prompt: '{args.prompt}'")
    try:
        raw_mission_dict = llm_parser.parse_prompt(args.prompt)
        logger.info(f"🧠 LLM Extracted JSON:\n{json.dumps(raw_mission_dict, indent=2)}")
    except Exception as e:
        logger.error(f"Failed to parse prompt: {e}")
        sys.exit(1)
        
    # 3. Validate JSON (Raw JSON dict -> Validated MissionPlan)
    try:
        logger.info("🛡️ Validating mission against safety constraints...")
        validated_plan = validator.validate(raw_mission_dict)
        logger.info("✅ Validation successful!")
    except Exception as e:
        logger.error(f"❌ Validation Error: {e}")
        sys.exit(1)

    # 4. Initialize Robot Controller
    try:
        squad_missions = ["formation", "split_patrol", "regroup"]
        is_squad = validated_plan.mission_type in squad_missions or validated_plan.agents is not None or validated_plan.agent_routes is not None

        if args.ros:
            if args.robot == "ebot":
                logger.info("🔌 Loading ebot Direct Controller...")
                from robot.ebot_controller import EbotRobotController
                robot = EbotRobotController()
            else:
                logger.info("🔌 Loading ROS 2 Nav2 Controller...")
                from robot.ros2_controller import ROS2Nav2Controller
                robot = ROS2Nav2Controller(wait_for_nav2=not is_squad)
        else:
            robot = MockRobotController()
            
        executor = MissionExecutor(robot=robot, waypoints_config=validator.waypoints)
    except Exception as e:
        logger.error(f"Failed to initialize robot controller: {e}")
        sys.exit(1)
        
    # 4. Execute Mission (Validated MissionPlan -> Physical Commands)
    try:
        executor.execute(validated_plan)
    except KeyboardInterrupt:
        logger.warning("🛑 Keyboard interrupt received! Stopping robot.")
        robot.stop()
        sys.exit(0)
    except Exception as e:
        logger.error(f"❌ Execution Error: {e}")
        robot.stop()
        sys.exit(1)

if __name__ == "__main__":
    main()
