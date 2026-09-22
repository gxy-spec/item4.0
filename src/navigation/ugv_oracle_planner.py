"""Generate an on-road UGV route only from a received target position."""

from __future__ import annotations

from typing import Any


def plan_route_from_message(
    world_map: Any,
    start_location: Any,
    target_xyz: list[float],
    standoff_m: float = 6.0,
    sampling_resolution_m: float = 2.0,
) -> list[list[float]]:
    import carla
    target_location = carla.Location(x=float(target_xyz[0]), y=float(target_xyz[1]), z=float(target_xyz[2]))
    target_waypoint = world_map.get_waypoint(
        target_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if target_waypoint is None:
        raise RuntimeError("Oracle target position cannot be projected to a driving lane")
    current = world_map.get_waypoint(
        start_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if current is None:
        raise RuntimeError("UGV start position cannot be projected to a driving lane")
    points: list[list[float]] = []
    visited: set[tuple[int, int, int]] = set()
    for _ in range(1200):
        location = current.transform.location
        points.append([float(location.x), float(location.y), float(location.z)])
        if location.distance(target_waypoint.transform.location) <= sampling_resolution_m * 1.25:
            break
        visited.add((int(current.road_id), int(current.lane_id), int(round(current.s))))
        candidates = list(current.next(sampling_resolution_m))
        unvisited = [
            waypoint
            for waypoint in candidates
            if (int(waypoint.road_id), int(waypoint.lane_id), int(round(waypoint.s))) not in visited
        ]
        if unvisited:
            candidates = unvisited
        if not candidates:
            raise RuntimeError("CARLA waypoint graph ended before reaching the Oracle target")
        current = min(
            candidates,
            key=lambda waypoint: waypoint.transform.location.distance(target_waypoint.transform.location),
        )
    if len(points) < 2:
        raise RuntimeError("CARLA waypoint planner returned an empty Oracle route")
    if current.transform.location.distance(target_waypoint.transform.location) > sampling_resolution_m * 2.0:
        raise RuntimeError("CARLA waypoint planner did not converge to the Oracle target lane")
    # Keep the road route through the target lane.  The controller applies the
    # requested standoff against the received position; trimming here would make
    # CARLA's generic route follower brake several metres before the safe goal.
    return points
