from pydantic import BaseModel, Field
from typing import Optional, List

class Waypoint(BaseModel):
    """
    Represents an individual 2D coordinate pose in space.
    """
    name: Optional[str] = Field(
        default=None, 
        description="Optional descriptive name for the waypoint"
    )
    x: float = Field(
        description="The X coordinate in the map frame (meters)"
    )
    y: float = Field(
        description="The Y coordinate in the map frame (meters)"
    )
    theta: float = Field(
        default=0.0, 
        description="Target orientation yaw angle (radians)"
    )

class AgentRoute(BaseModel):
    """
    Represents an explicit route assignment for a single robot agent.
    """
    agent_id: str = Field(description="The namespaced ID of the robot, e.g., 'tb3_0', 'tb3_1'")
    route: str = Field(description="The pre-configured route name for this agent, e.g., 'top_side'")
    loops: int = Field(default=1, description="Number of times to loop this route")

class MissionPlan(BaseModel):
    """
    Pydantic schema defining the structured JSON expected from the LLM.
    This guarantees type safety and correct data extraction.
    """
    mission_type: str = Field(
        description="The type of mission, e.g., 'patrol', 'inspect', 'deliver', 'navigate', 'follow', 'split_patrol', 'formation'"
    )
    route: Optional[str] = Field(
        default=None,
        description="The name of the pre-configured route to follow (optional if custom waypoints are given)"
    )
    waypoints: Optional[List[Waypoint]] = Field(
        default=None,
        description="List of custom coordinate waypoints to execute (optional if a route is given)"
    )
    loops: int = Field(
        default=1, 
        description="Number of times to execute the route or waypoints"
    )
    speed: Optional[float] = Field(
        default=None, 
        description="The speed of the robot in meters per second"
    )
    return_home: bool = Field(
        default=True, 
        description="Whether the robot should return home after the mission"
    )
    target_object: Optional[str] = Field(
        default=None,
        description="The target object or color to search for and follow, e.g., 'red', 'green', 'person', 'chair'"
    )
    formation_type: Optional[str] = Field(
        default=None,
        description="The multi-agent formation type, e.g., 'wedge', 'line', 'column'"
    )
    spacing: float = Field(
        default=1.0,
        description="The distance spacing between agents in the formation (meters)"
    )
    agents: Optional[List[str]] = Field(
        default=None,
        description="List of robot agent namespaced IDs involved in the mission, e.g., ['tb3_0', 'tb3_1']"
    )
    agent_routes: Optional[List[AgentRoute]] = Field(
        default=None,
        description="Explicit route assignments for individual agents"
    )

