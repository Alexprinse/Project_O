import math
from typing import List, Tuple, Dict

def get_slot_offsets(formation_type: str, num_agents: int, spacing: float) -> List[Tuple[float, float]]:
    """
    Returns the (x, y) offsets of each slot in the formation coordinate frame.
    Formation frame: +x is forward (direction of travel), +y is to the left.
    """
    formation_type = formation_type.lower()
    
    if formation_type == "wedge":
        # Slot 0 is leader, Slot 1 is left back, Slot 2 is right back
        offsets = [(0.0, 0.0)]
        if num_agents > 1:
            offsets.append((-spacing, spacing))
        if num_agents > 2:
            offsets.append((-spacing, -spacing))
        # Extend if more than 3 agents
        for i in range(3, num_agents):
            side = 1 if i % 2 == 1 else -1
            row = (i - 1) // 2 + 1
            offsets.append((-row * spacing, side * row * spacing))
        return offsets
        
    elif formation_type == "line":
        # Perpendicular to motion.
        if num_agents == 2:
            return [(0.0, spacing / 2.0), (0.0, -spacing / 2.0)]
        else:
            # Slot 0 is center, Slot 1 is left, Slot 2 is right, etc.
            offsets = [(0.0, 0.0)]
            for i in range(1, num_agents):
                side = 1 if i % 2 == 1 else -1
                step = (i - 1) // 2 + 1
                offsets.append((0.0, side * step * spacing))
            return offsets
            
    elif formation_type == "column":
        # Straight line behind leader
        return [(-i * spacing, 0.0) for i in range(num_agents)]
        
    else:
        # Default to Column if unknown
        return [(-i * spacing, 0.0) for i in range(num_agents)]

def project_slots_to_world(
    anchor: Tuple[float, float], 
    heading: float, 
    offsets: List[Tuple[float, float]]
) -> List[Tuple[float, float, float]]:
    """
    Projects formation-frame offsets into world coordinates relative to an anchor pose.
    anchor: (Ax, Ay)
    heading: psi (radians)
    returns: List of (wx, wy, w_theta) target coordinates
    """
    Ax, Ay = anchor
    world_slots = []
    
    for ox, oy in offsets:
        wx = Ax + ox * math.cos(heading) - oy * math.sin(heading)
        wy = Ay + ox * math.sin(heading) + oy * math.cos(heading)
        world_slots.append((wx, wy, heading))
        
    return world_slots

def assign_agents_to_slots(
    agent_poses: Dict[str, Tuple[float, float]], 
    slot_positions: List[Tuple[float, float, float]]
) -> Dict[str, Tuple[float, float, float]]:
    """
    Assigns agents to slots using a greedy nearest-neighbor solver to minimize travel distance
    and prevent path crossing.
    agent_poses: Dict of agent_id -> (x, y) current poses
    slot_positions: List of (x, y, theta) target world positions
    returns: Dict of agent_id -> (x, y, theta) target slots
    """
    assigned = {}
    remaining_slots = list(slot_positions)
    agents = list(agent_poses.keys())
    
    # Simple greedy solver: for each agent, find the closest remaining slot
    for agent_id in agents:
        if not remaining_slots:
            break
        ax, ay = agent_poses[agent_id]
        
        # Find closest slot
        best_idx = 0
        min_dist = float("inf")
        
        for idx, (sx, sy, _) in enumerate(remaining_slots):
            dist = math.hypot(sx - ax, sy - ay)
            if dist < min_dist:
                min_dist = dist
                best_idx = idx
                
        assigned[agent_id] = remaining_slots.pop(best_idx)
        
    return assigned
