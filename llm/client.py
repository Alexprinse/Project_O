import os
import json
import logging
from .prompts import get_system_prompt
from validator.schema import MissionPlan

logger = logging.getLogger(__name__)

class MissionParser:
    def __init__(self, use_mock: bool = False, model_name: str = "gemini-2.5-flash", allowed_routes: list[str] = None, known_waypoints: dict = None):
        self.use_mock = use_mock
        self.model_name = model_name
        self.allowed_routes = allowed_routes
        self.known_waypoints = known_waypoints
        self.client = None
        
        if not self.use_mock:
            try:
                from google import genai
                from google.genai import types
                api_key = os.environ.get("GEMINI_API_KEY")
                if api_key:
                    self.client = genai.Client()
                else:
                    logger.warning("GEMINI_API_KEY not found in environment. Falling back to Mock parser.")
                    self.use_mock = True
            except ImportError:
                logger.warning("google-genai not installed. Falling back to Mock parser.")
                self.use_mock = True
                
    def parse_prompt(self, user_prompt: str) -> dict:
        """
        Parses the user prompt into a dictionary matching the MissionPlan schema.
        """
        if self.use_mock:
            return self._mock_parse(user_prompt)
            
        return self._llm_parse(user_prompt)
        
    def _llm_parse(self, user_prompt: str) -> dict:
        from google.genai import types
        
        system_instruction = get_system_prompt(self.allowed_routes, self.known_waypoints)
        logger.info(f"Sending prompt to LLM ({self.model_name})...")
        response = self.client.models.generate_content(
            model=self.model_name,
            contents=user_prompt,
            config=types.GenerateContentConfig(
                system_instruction=system_instruction,
                response_mime_type="application/json",
                response_schema=MissionPlan,
                temperature=0.0,
            ),
        )
        
        raw_json = response.text
        try:
            return json.loads(raw_json)
        except json.JSONDecodeError as e:
            logger.error(f"Failed to decode LLM response: {raw_json}")
            raise ValueError("LLM returned invalid JSON.") from e
 
    def _mock_parse(self, user_prompt: str) -> dict:
        """
        A naive keyword-based parser for offline testing.
        """
        logger.info("Using Mock parser for prompt...")
        prompt_lower = user_prompt.lower()
        
        plan = {
            "mission_type": "patrol",
            "route": None,
            "waypoints": None,
            "loops": 1,
            "speed": None,
            "return_home": True,
            "target_object": None,
            "formation_type": None,
            "spacing": 1.0,
            "agents": None,
            "agent_routes": None
        }
        
        # Check for known named waypoints in the prompt
        found_wps = []
        if self.known_waypoints:
            for name, wp in self.known_waypoints.items():
                if name in prompt_lower:
                    found_wps.append({"name": name, "x": wp["x"], "y": wp["y"], "theta": wp.get("theta", 0.0)})
        
        # Check for coordinate navigation
        import re
        x_match = re.search(r"x\s*=\s*(-?\d+\.?\d*)", prompt_lower)
        y_match = re.search(r"y\s*=\s*(-?\d+\.?\d*)", prompt_lower)
        coords_match = re.search(r"(?:navigate to|go to)\s+(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)", prompt_lower)
        
        if found_wps:
            plan["mission_type"] = "navigate"
            plan["route"] = None
            plan["waypoints"] = found_wps
        elif x_match and y_match:
            x_val = float(x_match.group(1))
            y_val = float(y_match.group(1))
            plan["mission_type"] = "navigate"
            plan["route"] = None
            plan["waypoints"] = [{"name": "target", "x": x_val, "y": y_val, "theta": 0.0}]
        elif coords_match:
            x_val = float(coords_match.group(1))
            y_val = float(coords_match.group(2))
            plan["mission_type"] = "navigate"
            plan["route"] = None
            plan["waypoints"] = [{"name": "target", "x": x_val, "y": y_val, "theta": 0.0}]
        else:
            if "inspect" in prompt_lower:
                plan["mission_type"] = "inspect"
                plan["route"] = "inspection_route"
            if "perimeter" in prompt_lower:
                plan["route"] = "perimeter"
            if "patrol" in prompt_lower:
                plan["mission_type"] = "patrol"
                plan["route"] = "warehouse_patrol"
            if "delivery" in prompt_lower:
                plan["mission_type"] = "deliver"
                plan["route"] = "delivery"
            if "top_side" in prompt_lower:
                plan["route"] = "top_side"
            if "bottom_side" in prompt_lower:
                plan["route"] = "bottom_side"
            
        # Check for follow mission
        if "follow" in prompt_lower or "track" in prompt_lower or "find" in prompt_lower:
            plan["mission_type"] = "follow"
            
            # Extract target
            for token in ["red", "green", "blue", "yellow", "person", "chair", "bottle", "car", "bus", "traffic light", "table", "box", "cone", "tree"]:
                if token in prompt_lower:
                    plan["target_object"] = token
                    break
            if not plan["target_object"]:
                plan["target_object"] = "red" # default target
                
        # Check for multi-agent formations
        if "wedge" in prompt_lower or "side-by-side" in prompt_lower or "line" in prompt_lower or "column" in prompt_lower:
            plan["mission_type"] = "formation"
            if "wedge" in prompt_lower:
                plan["formation_type"] = "wedge"
            elif "side-by-side" in prompt_lower or "line" in prompt_lower:
                plan["formation_type"] = "line"
            elif "column" in prompt_lower:
                plan["formation_type"] = "column"
                
            # Default agents
            agents = ["tb3_0", "tb3_1", "tb3_2"]
            if "robot 1" in prompt_lower and "robot 2" in prompt_lower and "robot 3" not in prompt_lower:
                agents = ["tb3_0", "tb3_1"]
            plan["agents"] = agents
            
        # Check for split patrol / routing
        if "split" in prompt_lower:
            plan["mission_type"] = "split_patrol"
            agents = ["tb3_0", "tb3_1", "tb3_2"]
            if "robot 1" in prompt_lower and "robot 2" in prompt_lower and "robot 3" not in prompt_lower:
                agents = ["tb3_0", "tb3_1"]
            plan["agents"] = agents
            
        if "robot 1 patrol top_side" in prompt_lower or "robot 1 patrol the top_side" in prompt_lower or \
           "robot 2 patrol bottom_side" in prompt_lower or "robot 2 patrol the bottom_side" in prompt_lower:
            plan["mission_type"] = "split_patrol"
            plan["agent_routes"] = []
            if "top_side" in prompt_lower:
                plan["agent_routes"].append({"agent_id": "tb3_0", "route": "top_side", "loops": 1})
            if "bottom_side" in prompt_lower:
                plan["agent_routes"].append({"agent_id": "tb3_1", "route": "bottom_side", "loops": 1})
            
        if "twice" in prompt_lower or "2 times" in prompt_lower:
            plan["loops"] = 2
        elif "thrice" in prompt_lower or "3 times" in prompt_lower:
            plan["loops"] = 3
            
        if "speed" in prompt_lower:
            words = prompt_lower.split()
            for i, word in enumerate(words):
                if "speed" in word and i + 1 < len(words):
                    try:
                        plan["speed"] = float(words[i+1])
                    except ValueError:
                        pass
                        
        if "do not return" in prompt_lower or "stay there" in prompt_lower:
            plan["return_home"] = False
            
        return plan

