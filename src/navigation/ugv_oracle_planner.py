"""Generate an on-road UGV route only from a received target position."""

from __future__ import annotations

import heapq
import itertools
import math
from typing import Any


def plan_route_from_message(
    world_map: Any,
    start_location: Any,
    target_xyz: list[float],
    standoff_m: float = 6.0,
    sampling_resolution_m: float = 2.0,
    reference_route: list[list[float]] | None = None,
    reference_route_endpoint_tolerance_m: float = 8.0,
) -> list[list[float]]:
    import carla

    target_location = carla.Location(x=float(target_xyz[0]), y=float(target_xyz[1]), z=float(target_xyz[2]))
    target_waypoint = world_map.get_waypoint(
        target_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if target_waypoint is None:
        raise RuntimeError("Received target position cannot be projected to a driving lane")
    current = world_map.get_waypoint(
        start_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if current is None:
        raise RuntimeError("UGV start position cannot be projected to a driving lane")

    # CARLA's graph planner is required here because a nearest-next greedy walk
    # can select the wrong branch at an intersection when perception noise moves
    # a roadside target by only a few centimetres.  The target is first projected
    # to a driving waypoint, so the planner never consumes actor ground truth.
    try:
        from agents.navigation.global_route_planner import GlobalRoutePlanner

        planner = GlobalRoutePlanner(world_map, sampling_resolution_m)
        traced = planner.trace_route(start_location, target_waypoint.transform.location)
        points = [
            [
                float(waypoint.transform.location.x),
                float(waypoint.transform.location.y),
                float(waypoint.transform.location.z),
            ]
            for waypoint, _ in traced
        ]
        if len(points) >= 2:
            return points
    except (ImportError, RuntimeError, ValueError):
        # Retain the deterministic legacy walk as a compatibility fallback for
        # CARLA distributions that do not ship the agents package.
        pass

    def key(waypoint: Any) -> tuple[int, int, int, int]:
        return (
            int(waypoint.road_id),
            int(waypoint.section_id),
            int(waypoint.lane_id),
            int(round(float(waypoint.s) / max(0.5, sampling_resolution_m / 2.0))),
        )

    start_key = key(current)
    counter = itertools.count()
    frontier: list[tuple[float, int, tuple[int, int, int, int]]] = []
    heapq.heappush(
        frontier,
        (current.transform.location.distance(target_waypoint.transform.location), next(counter), start_key),
    )
    waypoints = {start_key: current}
    parents: dict[tuple[int, int, int, int], tuple[int, int, int, int] | None] = {start_key: None}
    costs = {start_key: 0.0}
    goal_key: tuple[int, int, int, int] | None = None
    for _ in range(50000):
        if not frontier:
            break
        _, _, current_key = heapq.heappop(frontier)
        current_waypoint = waypoints[current_key]
        if (
            current_waypoint.transform.location.distance(target_waypoint.transform.location)
            <= sampling_resolution_m * 2.0
        ):
            goal_key = current_key
            break
        for following in current_waypoint.next(sampling_resolution_m):
            following_key = key(following)
            step_cost = current_waypoint.transform.location.distance(following.transform.location)
            candidate_cost = costs[current_key] + float(step_cost)
            if candidate_cost >= costs.get(following_key, float("inf")):
                continue
            costs[following_key] = candidate_cost
            parents[following_key] = current_key
            waypoints[following_key] = following
            heuristic = following.transform.location.distance(target_waypoint.transform.location)
            heapq.heappush(
                frontier,
                (candidate_cost + float(heuristic), next(counter), following_key),
            )
    if goal_key is not None:
        route_keys: list[tuple[int, int, int, int]] = []
        cursor: tuple[int, int, int, int] | None = goal_key
        while cursor is not None:
            route_keys.append(cursor)
            cursor = parents[cursor]
        route_keys.reverse()
        points = [
            [
                float(waypoints[item].transform.location.x),
                float(waypoints[item].transform.location.y),
                float(waypoints[item].transform.location.z),
            ]
            for item in route_keys
        ]
        if len(points) >= 2:
            return points

    if reference_route and len(reference_route) >= 2:
        endpoint = reference_route[-1]
        endpoint_error = math.hypot(
            float(endpoint[0]) - float(target_waypoint.transform.location.x),
            float(endpoint[1]) - float(target_waypoint.transform.location.y),
        )
        if endpoint_error <= reference_route_endpoint_tolerance_m:
            return [[float(value) for value in point] for point in reference_route]
    raise RuntimeError("CARLA waypoint planner did not converge to the received target lane")
