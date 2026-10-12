"""Offline candidate search on the directed Town10HD survey topology.

This is only a route-design aid; CARLA's full lane planner must validate picks.
"""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import networkx as nx


SURVEY = Path("E:/CarlaAirData/CityInspection_GOC/e0_platform/scene_definition_v1")
SEGMENTS = json.loads((SURVEY / "town10hd_topology.json").read_text(encoding="utf-8"))["segments"]
with (SURVEY / "town10hd_spawn_points.csv").open(encoding="utf-8-sig", newline="") as source:
    SPAWNS = {int(row["spawn_point_id"]): row for row in csv.DictReader(source)}


def pos(spawn_id):
    row = SPAWNS[spawn_id]
    return (float(row["x"]), float(row["y"]))


def xy(point):
    return tuple(round(float(value), 4) for value in point[:2])


def projection(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    denominator = dx * dx + dy * dy
    t = 0.0 if denominator == 0 else max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / denominator))
    return (start[0] + t * dx, start[1] + t * dy), t


def point_to_polyline(point, polyline):
    def segment(a, b):
        projected, _ = projection(point, a, b)
        return math.dist(point, projected)

    return min(segment(a, b) for a, b in zip(polyline, polyline[1:]))


def build():
    graph = nx.DiGraph()
    edge_points = defaultdict(list)
    for index, (raw_a, raw_b) in enumerate(SEGMENTS):
        a, b = xy(raw_a), xy(raw_b)
        if a == b:
            continue
        graph.add_edge(a, b, weight=math.dist(a, b))
    for spawn_id, row in SPAWNS.items():
        point = pos(spawn_id)
        yaw = math.radians(float(row["yaw"]))
        heading = (math.cos(yaw), math.sin(yaw))
        candidates = []
        for a, b in graph.edges:
            projected, t = projection(point, a, b)
            dx, dy = b[0] - a[0], b[1] - a[1]
            seglen = math.hypot(dx, dy)
            alignment = (dx * heading[0] + dy * heading[1]) / seglen
            candidates.append((math.dist(point, projected) + (0 if alignment > 0.5 else 100), a, b, projected, t))
        distance, a, b, projected, t = min(candidates)
        if distance >= 100:
            raise ValueError(f"No aligned lane for spawn {spawn_id}")
        edge_points[(a, b)].append((t, ("spawn", spawn_id), point, projected, distance))
    for (a, b), points in edge_points.items():
        graph.remove_edge(a, b)
        chain = [(0.0, a, a)] + [(t, label, projected) for t, label, _, projected, _ in points] + [(1.0, b, b)]
        chain.sort(key=lambda item: item[0])
        for (_, first, first_xy), (_, second, second_xy) in zip(chain, chain[1:]):
            graph.add_edge(first, second, weight=max(1e-5, math.dist(first_xy, second_xy)))
    return graph, {("spawn", spawn_id): pos(spawn_id) for spawn_id in SPAWNS}


def inspect(name, bounds, start_ids=None, *, minimum_prefix=8, maximum_prefix=90):
    graph, spawn_xy = build()
    x0, x1, y0, y1 = bounds
    ids = [key for key, point in spawn_xy.items() if x0 <= point[0] <= x1 and y0 <= point[1] <= y1]
    starts = [("spawn", sid) for sid in start_ids] if start_ids else ids
    options = []
    for start in starts:
        all_paths = nx.single_source_dijkstra_path(graph, start, weight="weight")
        for goal in ids:
            if goal == start or goal not in all_paths:
                continue
            patrol_nodes = all_paths[goal]
            patrol = [spawn_xy[node] if node in spawn_xy else node for node in patrol_nodes]
            patrol_length = sum(math.dist(a, b) for a, b in zip(patrol, patrol[1:]))
            if not 40 <= patrol_length <= 200 or any(not(x0 - 5 <= x <= x1 + 5 and y0 - 5 <= y <= y1 + 5) for x, y in patrol):
                continue
            for target in ids:
                if target in (start, goal) or target not in all_paths:
                    continue
                target_nodes = all_paths[target]
                target_path = [spawn_xy[node] if node in spawn_xy else node for node in target_nodes]
                target_length = sum(math.dist(a, b) for a, b in zip(target_path, target_path[1:]))
                if not 45 <= target_length <= 220 or any(not(x0 - 5 <= x <= x1 + 5 and y0 - 5 <= y <= y1 + 5) for x, y in target_path):
                    continue
                shared_nodes = 0
                for a, b in zip(patrol_nodes, target_nodes):
                    if a != b:
                        break
                    shared_nodes += 1
                shared_path = patrol[:shared_nodes]
                shared_length = sum(math.dist(a, b) for a, b in zip(shared_path, shared_path[1:]))
                separation = point_to_polyline(spawn_xy[target], patrol)
                if not(minimum_prefix <= shared_length <= maximum_prefix and separation >= 25):
                    continue
                score = separation + 0.3 * shared_length - 0.08 * patrol_length
                options.append((score, start[1], goal[1], target[1], round(shared_length, 1), round(separation, 1), round(patrol_length, 1), round(target_length, 1)))
    options.sort(reverse=True)
    print(name, "candidates", len(options))
    for row in options[:20]:
        print("  ", row)


if __name__ == "__main__":
    inspect("north", (-85, 42, 78, 147), [22, 80, 144, 16, 17, 75])
    inspect("east", (20, 116, -31, 99), [108, 73, 69, 70, 78, 79, 133, 149])
    inspect("west", (-120, -38, -36, 84), [10, 12, 13, 6, 7, 8, 9, 91, 141])
    inspect("whole", (-120, 116, -76, 147), [80, 73, 108, 37, 22, 69], maximum_prefix=120)
