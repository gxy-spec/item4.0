from __future__ import annotations

import math
import sys
from pathlib import Path


SCRIPTS_ROOT = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from run_stage1_dynamic import (
    bounded_nearest_route_index,
    cumulative_distance,
    interpolate_polyline,
    remaining_route_distance,
    route_completion_metrics,
)


def test_bounded_nearest_route_index_does_not_jump_across_crossing_route() -> None:
    route = [[float(index), 0.0, 0.0] for index in range(30)]
    route.extend([[0.0, 0.1, 0.0], [-1.0, 0.1, 0.0]])
    assert bounded_nearest_route_index(route, [0.0, 0.1], cursor_index=5, forward_window=8) == 5


def test_remaining_route_distance_uses_unconsumed_route_not_euclidean_shortcut() -> None:
    route = [[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [100.0, 1.0, 0.0], [0.0, 1.0, 0.0]]
    assert remaining_route_distance(route, 0, [0.0, 0.1, 0.0]) > 200.0
    assert remaining_route_distance(route, 3, [0.0, 0.1, 0.0]) < 1.0


def test_interpolate_polyline_uses_metric_distance() -> None:
    points = [[0.0, 0.0, 5.0], [10.0, 0.0, 5.0], [10.0, 10.0, 5.0]]
    xyz, yaw = interpolate_polyline(points, 15.0)
    assert xyz == [10.0, 5.0, 5.0]
    assert yaw == 90.0


def test_interpolate_polyline_clamps_at_route_end() -> None:
    xyz, yaw = interpolate_polyline([[1.0, 2.0, 3.0], [4.0, 6.0, 3.0]], 99.0)
    assert xyz == [4.0, 6.0, 3.0]
    assert math.isclose(yaw, math.degrees(math.atan2(4.0, 3.0)))


def test_cumulative_distance() -> None:
    assert cumulative_distance([[0.0, 0.0, 0.0], [3.0, 4.0, 0.0], [3.0, 8.0, 0.0]]) == 9.0


def test_route_completion_counts_lawnmower_foldbacks() -> None:
    route = [
        [0.0, 0.0, 5.0],
        [10.0, 0.0, 5.0],
        [10.0, 2.0, 5.0],
        [0.0, 2.0, 5.0],
        [0.0, 4.0, 5.0],
        [10.0, 4.0, 5.0],
    ]
    metrics = route_completion_metrics(route, route, waypoint_radius_m=0.1)
    assert metrics == {"waypoints_reached": 6, "segments_completed": 5, "foldbacks_completed": 2}
