"""Spatial memory for the mission: observed lines, obstacles, balls, graph."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class MapPoint:
    x_m: float
    y_m: float
    kind: str


@dataclass
class GraphNode:
    x_m: float
    y_m: float


class LineGraph:
    """Sparse graph of observed line points, used for return-home routing."""

    def __init__(
        self,
        merge_distance_m: float = 0.06,
        obstacle_block_radius_m: float = 0.12,
    ) -> None:
        self.merge_distance_m = merge_distance_m
        self.obstacle_block_radius_m = obstacle_block_radius_m
        self.nodes: list[GraphNode] = []
        self.edges: dict[int, dict[int, float]] = {}
        self.last_node: int | None = None
        self.home_node: int | None = None
        self.obstacle_nodes: set[int] = set()

    def mark_home(self, x_m: float, y_m: float) -> None:
        self.home_node = self._find_or_add_node(x_m, y_m)

    def add_line_point(self, x_m: float, y_m: float) -> int:
        node_idx = self._find_or_add_node(x_m, y_m)
        if self.last_node is not None and self.last_node != node_idx:
            self._add_edge(self.last_node, node_idx)
        self.last_node = node_idx
        return node_idx

    def add_obstacle(self, x_m: float, y_m: float) -> None:
        node_idx = self._find_or_add_node(x_m, y_m)
        self.obstacle_nodes.add(node_idx)

    def find_nearest_node(self, x_m: float, y_m: float) -> int | None:
        best_idx = None
        best_dist = None
        for idx, node in enumerate(self.nodes):
            dist = math.hypot(node.x_m - x_m, node.y_m - y_m)
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_idx = idx
        return best_idx

    def shortest_path(self, start: int, goal: int) -> list[int]:
        if start == goal:
            return [start]
        if start in self.obstacle_nodes or goal in self.obstacle_nodes:
            return []

        unvisited: set[int] = set(range(len(self.nodes)))
        dist: dict[int, float] = {start: 0.0}
        prev: dict[int, int] = {}

        while unvisited:
            current = None
            current_dist = None
            for idx in unvisited:
                if idx not in dist:
                    continue
                if current_dist is None or dist[idx] < current_dist:
                    current = idx
                    current_dist = dist[idx]

            if current is None:
                break
            if current == goal:
                break

            unvisited.remove(current)
            for neighbor, cost in self.edges.get(current, {}).items():
                if neighbor not in unvisited:
                    continue
                if neighbor in self.obstacle_nodes:
                    continue
                candidate = dist[current] + cost
                if candidate < dist.get(neighbor, float("inf")):
                    dist[neighbor] = candidate
                    prev[neighbor] = current

        if goal not in dist:
            return []

        path = [goal]
        while path[-1] != start:
            path.append(prev[path[-1]])
        path.reverse()
        return path

    def _find_or_add_node(self, x_m: float, y_m: float) -> int:
        for idx, node in enumerate(self.nodes):
            if math.hypot(node.x_m - x_m, node.y_m - y_m) <= self.merge_distance_m:
                return idx
        idx = len(self.nodes)
        self.nodes.append(GraphNode(x_m, y_m))
        self.edges[idx] = {}
        return idx

    def _add_edge(self, a: int, b: int) -> None:
        node_a = self.nodes[a]
        node_b = self.nodes[b]
        cost = math.hypot(node_b.x_m - node_a.x_m, node_b.y_m - node_a.y_m)
        self.edges[a][b] = cost
        self.edges[b][a] = cost


__all__ = ["MapPoint", "GraphNode", "LineGraph"]
