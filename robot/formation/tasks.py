import math
import logging
from typing import List, Tuple, Dict
from concurrent.futures import ThreadPoolExecutor, wait

from robot.formation.formations import assign_agents_to_slots

logger = logging.getLogger(__name__)

def split_route_greedy(agent_poses: Dict[str, Tuple[float, float, float]], waypoints: List[Tuple[float, float, float]]) -> Dict[str, List[Tuple[float, float, float]]]:
    """
    Partitions a list of waypoints among agents to minimize travel and prevent crossed paths.
    agent_poses: Dict of agent_id -> (x, y, theta) starting pose
    waypoints: List of (x, y, theta) waypoints
    returns: Dict of agent_id -> List of (x, y, theta) waypoints
    """
    assigned = {aid: [] for aid in agent_poses.keys()}
    unassigned = list(waypoints)
    
    # Keep track of the current "tail" position of each agent's path
    current_positions = {aid: pose for aid, pose in agent_poses.items()}
    
    while unassigned:
        best_agent = None
        best_wp_idx = None
        min_dist = float("inf")
        
        # Proximity pairing: find the closest unassigned waypoint to any agent's current path tail
        for aid in agent_poses.keys():
            cx, cy, _ = current_positions[aid]
            for idx, wp in enumerate(unassigned):
                dist = math.hypot(wp[0] - cx, wp[1] - cy)
                if dist < min_dist:
                    min_dist = dist
                    best_agent = aid
                    best_wp_idx = idx
                    
        if best_agent is not None and best_wp_idx is not None:
            wp = unassigned.pop(best_wp_idx)
            assigned[best_agent].append(wp)
            # Update the tail of this agent to the newly assigned waypoint
            current_positions[best_agent] = wp
            
    return assigned

def execute_split_patrol(manager, route_waypoints: List[Tuple[float, float, float]], speed: float) -> bool:
    """
    Splits a single list of waypoints spatially among agents and executes them concurrently.
    """
    logger.info("⚡ Executing Split Patrol (Implicit Routing)...")
    agent_poses = manager.get_agent_poses()
    
    # Partition waypoints
    partitions = split_route_greedy(agent_poses, route_waypoints)
    
    # Print partitioning decisions
    for aid, wps in partitions.items():
        logger.info(f"   Agent '{aid}' assigned {len(wps)} waypoints: {[w[0:2] for w in wps]}")
        
    # Execute in parallel
    success = True
    with ThreadPoolExecutor(max_workers=len(manager.agent_ids)) as executor:
        futures = []
        for aid in manager.agent_ids:
            wps = partitions[aid]
            if not wps:
                continue
            # Run sequential waypoints for this specific agent in its thread
            futures.append(
                executor.submit(_run_agent_sequence, manager.agents[aid], wps, speed)
            )
        # Barrier wait
        results = [f.result() for f in futures]
        if not all(results):
            success = False
            
    return success

def execute_explicit_patrol(manager, agent_routes: List[dict], waypoints_config: dict, speed: float) -> bool:
    """
    Executes explicit route zone assignments concurrently for the agents.
    """
    logger.info("⚡ Executing Explicit Route Patrol (Zone Assignment)...")
    
    success = True
    with ThreadPoolExecutor(max_workers=len(manager.agent_ids)) as executor:
        futures = []
        for route_info in agent_routes:
            aid = route_info["agent_id"]
            route_name = route_info["route"]
            loops = route_info.get("loops", 1)
            
            if aid not in manager.agents:
                logger.warning(f"Skipping assignment: Agent '{aid}' is not active in this squad.")
                continue
                
            # Load route waypoints from configuration
            if "routes" in waypoints_config and route_name in waypoints_config["routes"]:
                raw_wps = waypoints_config["routes"][route_name]
                wps = [(wp["x"], wp["y"], wp.get("theta", 0.0)) for wp in raw_wps]
            else:
                logger.error(f"Route '{route_name}' not found in waypoint configuration.")
                return False
                
            logger.info(f"   Assigning Agent '{aid}' to route '{route_name}' ({len(wps)} waypoints, loops: {loops})")
            
            # Submit task to executor thread
            futures.append(
                executor.submit(_run_agent_sequence, manager.agents[aid], wps, speed, loops)
            )
            
        results = [f.result() for f in futures]
        if not all(results):
            success = False
            
    return success

def execute_regroup(manager, regroup_at: Tuple[float, float], speed: float) -> bool:
    """
    Drives all agents to a single rendezvous point, automatically adding spacing offsets
    to prevent collision at the exact same target position.
    regroup_at: (rx, ry)
    """
    rx, ry = regroup_at
    logger.info(f"🔄 Regrouping squad at rendezvous point: ({rx}, {ry})...")
    
    # Calculate offset slots around regroup point (Wedge shape)
    final_slots = [
        (rx, ry, 0.0),
        (rx - 1.0, ry + 1.0, 0.0),
        (rx - 1.0, ry - 1.0, 0.0)
    ]
    
    # Truncate slots to match number of active agents
    final_slots = final_slots[:len(manager.agent_ids)]
    
    agent_poses = manager.get_agent_poses()
    assignments = assign_agents_to_slots(
        {aid: (p[0], p[1]) for aid, p in agent_poses.items()},
        final_slots
    )
    
    success = True
    with ThreadPoolExecutor(max_workers=len(manager.agent_ids)) as executor:
        futures = []
        for aid, (sx, sy, stheta) in assignments.items():
            logger.info(f"   Agent '{aid}' navigating to regroup slot: ({sx:.2f}, {sy:.2f})")
            futures.append(
                executor.submit(manager.agents[aid].navigate_to, "regroup", sx, sy, stheta, speed)
            )
        results = [f.result() for f in futures]
        if not all(results):
            success = False
            
    return success

def _run_agent_sequence(agent, waypoints: List[Tuple[float, float, float]], speed: float, loops: int = 1) -> bool:
    """
    Helper function running in a background thread to navigate a single agent
    sequentially through its assigned queue of waypoints.
    """
    for loop in range(1, loops + 1):
        for idx, wp in enumerate(waypoints):
            wx, wy, wtheta = wp
            target_name = f"wp_{idx}"
            success = agent.navigate_to(target_name, wx, wy, wtheta, speed)
            if not success:
                logger.error(f"❌ Agent failed to reach waypoint '{target_name}' at ({wx}, {wy}) during loop {loop}.")
                return False
    return True
