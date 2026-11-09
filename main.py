#!/usr/bin/env python3
"""Fly-in Drone Routing Simulation System.

This module provides a complete object-oriented drone simulation system,
including map parsing with strict error validation, space-time pathfinding,
turn-by-turn simulation output, and optional capacity tracking.
"""

import argparse
from collections import defaultdict
import enum
import heapq
from itertools import count
import sys
from typing import Optional


class ZoneType(enum.Enum):
    """Enumeration of valid zone types and movement turn costs."""

    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"

    @property
    def cost(self) -> float:
        """Return the number of turns required to enter this zone.

        Returns:
            float: Turn cost associated with entering this zone type.
        """
        if self == ZoneType.NORMAL:
            return 1.0
        if self == ZoneType.PRIORITY:
            return 1.0
        if self == ZoneType.RESTRICTED:
            return 2.0
        return float("inf")


class Zone:
    """Represents a zone/hub in the drone network."""

    def __init__(
        self,
        name: str,
        coord: tuple[int, int],
        zone_type: ZoneType = ZoneType.NORMAL,
        color: str = "none",
        max_drones: int = 1,
    ) -> None:
        """Initialize a Zone instance."""
        self.name: str = name
        self.coord: tuple[int, int] = coord
        self.zone_type: ZoneType = zone_type
        self.color: str = color
        self.max_drones: int = max_drones

    def __repr__(self) -> str:
        """Return string representation of Zone."""
        return (
            f"Zone({self.name}, coord={self.coord}, "
            f"type={self.zone_type.value}, max_drones={self.max_drones})"
        )


class Connection:
    """Represents a bidirectional connection between two zones."""

    def __init__(
        self, zone1: str, zone2: str, max_link_capacity: int = 1
    ) -> None:
        """Initialize a Connection instance."""
        self.zone1: str = zone1
        self.zone2: str = zone2
        self.max_link_capacity: int = max_link_capacity

    @property
    def key(self) -> tuple[str, str]:
        """Return normalized key for undirected edge comparisons."""
        pair = sorted([self.zone1, self.zone2])
        return (pair[0], pair[1])

    def __repr__(self) -> str:
        """Return string representation of Connection."""
        return (
            f"Connection({self.zone1}-{self.zone2}, "
            f"capacity={self.max_link_capacity})"
        )


class MapConfig:
    """Holds validated network map config and graph structure."""

    def __init__(
        self,
        nb_drones: int,
        start_hub: str,
        end_hub: str,
        zones: dict[str, Zone],
        connections: list[Connection],
    ) -> None:
        """Initialize MapConfig instance."""
        self.nb_drones: int = nb_drones
        self.start_hub: str = start_hub
        self.end_hub: str = end_hub
        self.zones: dict[str, Zone] = zones
        self.connections: list[Connection] = connections
        self.adj: dict[str, list[str]] = defaultdict(list)
        self.conn_map: dict[tuple[str, str], Connection] = {}

        for conn in connections:
            self.adj[conn.zone1].append(conn.zone2)
            self.adj[conn.zone2].append(conn.zone1)
            self.conn_map[conn.key] = conn


class MapParser:
    """Parser class for loading and validating map files."""

    @staticmethod
    def _parse_metadata(meta_str: str, line_num: int) -> dict[str, str]:
        """Parse bracketed metadata string into a dictionary."""
        metadata: dict[str, str] = {}
        content = meta_str.strip()
        if content.startswith("[") and content.endswith("]"):
            content = content[1:-1]
        else:
            raise ValueError(f"Line {line_num}: Malformed metadata brackets")

        tokens = content.replace(",", " ").split()
        for token in tokens:
            if "=" in token:
                key, val = token.split("=", 1)
                metadata[key.strip()] = val.strip()
            else:
                metadata[token.strip()] = "true"
        return metadata

    @staticmethod
    def parse_line(
        line_val: str, line_num: int
    ) -> tuple[str, list[str], dict[str, str]]:
        """Parse line into name, args, and metadata dictionary."""
        if "[" in line_val:
            if not line_val.endswith("]"):
                raise ValueError(
                    f"Line {line_num}: Unclosed metadata bracket"
                )
            main_part, meta_part = line_val.rsplit("[", 1)
            meta_dict = MapParser._parse_metadata("[" + meta_part, line_num)
        else:
            main_part = line_val
            meta_dict = {}

        args = main_part.strip().split()
        if not args:
            raise ValueError(f"Line {line_num}: Missing zone/connection name")
        name = args[0]
        return name, args[1:], meta_dict

    def parse_file(self, filepath: str) -> MapConfig:
        """Read and validate a map configuration file."""
        nb_drones: Optional[int] = None
        start_hub: Optional[str] = None
        end_hub: Optional[str] = None
        zones: dict[str, Zone] = {}
        connections: list[Connection] = []
        seen_connections: set[tuple[str, str]] = set()
        seen_directive = False

        with open(filepath, "r", encoding="utf-8") as f:
            for line_num, line in enumerate(f, 1):
                raw_line = line.strip()
                if not raw_line or raw_line.startswith("#"):
                    continue

                if ":" not in raw_line:
                    raise ValueError(
                        f"Line {line_num}: Missing colon separator"
                    )

                key_part, val_part = raw_line.split(":", 1)
                key = key_part.strip()
                val = val_part.strip()

                if not seen_directive:
                    if key != "nb_drones":
                        raise ValueError(
                            f"Line {line_num}: first directive must be "
                            "'nb_drones'"
                        )
                    seen_directive = True

                if key == "nb_drones":
                    if nb_drones is not None:
                        raise ValueError(
                            f"Line {line_num}: Duplicate 'nb_drones'"
                        )
                    if not val.isdigit() or int(val) <= 0:
                        raise ValueError(
                            f"Line {line_num}: 'nb_drones' must be positive"
                        )
                    nb_drones = int(val)

                elif key in ("start_hub", "end_hub", "hub"):
                    name, args, meta = self.parse_line(val, line_num)
                    if "-" in name or " " in name:
                        raise ValueError(
                            f"Line {line_num}: Zone '{name}' cannot have "
                            "dashes or spaces"
                        )
                    if name in zones:
                        raise ValueError(
                            f"Line {line_num}: Duplicate zone name '{name}'"
                        )

                    if len(args) < 2:
                        raise ValueError(
                            f"Line {line_num}: Missing coords for '{name}'"
                        )
                    if len(args) > 2:
                        raise ValueError(
                            f"Line {line_num}: Unexpected extra tokens "
                            f"for '{name}'"
                        )
                    try:
                        x, y = int(args[0]), int(args[1])
                    except ValueError:
                        raise ValueError(
                            f"Line {line_num}: Coords must be integers"
                        )

                    z_type_str = meta.get("zone", "normal")
                    try:
                        z_type = ZoneType(z_type_str)
                    except ValueError:
                        raise ValueError(
                            f"Line {line_num}: Invalid zone type "
                            f"'{z_type_str}'"
                        )

                    max_d = 1
                    if key == "hub" and "max_drones" in meta:
                        if (
                            not meta["max_drones"].isdigit()
                            or int(meta["max_drones"]) <= 0
                        ):
                            raise ValueError(
                                f"Line {line_num}: 'max_drones' must be "
                                "positive integer"
                            )
                        max_d = int(meta["max_drones"])

                    color = meta.get("color", "none")

                    if key == "start_hub":
                        if start_hub is not None:
                            raise ValueError(
                                f"Line {line_num}: Multiple 'start_hub'"
                            )
                        start_hub = name
                    elif key == "end_hub":
                        if end_hub is not None:
                            raise ValueError(
                                f"Line {line_num}: Multiple 'end_hub'"
                            )
                        end_hub = name

                    zones[name] = Zone(
                        name,
                        (x, y),
                        zone_type=z_type,
                        color=color,
                        max_drones=max_d,
                    )

                elif key == "connection":
                    conn_str, args, meta = self.parse_line(val, line_num)
                    if args:
                        raise ValueError(
                            f"Line {line_num}: Unexpected extra tokens "
                            "for connection"
                        )
                    if "-" not in conn_str:
                        raise ValueError(
                            f"Line {line_num}: Conn must be 'zone1-zone2'"
                        )
                    z1, z2 = conn_str.split("-", 1)
                    if z1 not in zones or z2 not in zones:
                        raise ValueError(
                            f"Line {line_num}: Undefined zone in conn"
                        )
                    if z1 == z2:
                        raise ValueError(
                            f"Line {line_num}: Self connection invalid"
                        )

                    norm_pair = (min(z1, z2), max(z1, z2))
                    if norm_pair in seen_connections:
                        raise ValueError(
                            f"Line {line_num}: Duplicate conn '{conn_str}'"
                        )
                    seen_connections.add(norm_pair)

                    link_cap = 1
                    if "max_link_capacity" in meta:
                        if (
                            not meta["max_link_capacity"].isdigit()
                            or int(meta["max_link_capacity"]) <= 0
                        ):
                            raise ValueError(
                                f"Line {line_num}: 'max_link_capacity' "
                                "must be positive integer"
                            )
                        link_cap = int(meta["max_link_capacity"])

                    connections.append(Connection(z1, z2, link_cap))

                else:
                    raise ValueError(
                        f"Line {line_num}: Unknown directive '{key}'"
                    )

        if nb_drones is None:
            raise ValueError("Missing 'nb_drones' declaration in map file")
        if start_hub is None:
            raise ValueError("Missing 'start_hub' declaration in map file")
        if end_hub is None:
            raise ValueError("Missing 'end_hub' declaration in map file")

        zones[start_hub].max_drones = nb_drones
        zones[end_hub].max_drones = nb_drones

        if zones[start_hub].zone_type == ZoneType.BLOCKED:
            raise ValueError(f"Start hub '{start_hub}' cannot be blocked")
        if zones[end_hub].zone_type == ZoneType.BLOCKED:
            raise ValueError(f"End hub '{end_hub}' cannot be blocked")

        config = MapConfig(nb_drones, start_hub, end_hub, zones, connections)
        self._validate_path_exists(config)
        return config

    @staticmethod
    def _validate_path_exists(config: MapConfig) -> None:
        """Perform BFS to verify reachable path between start and end hub."""
        queue: list[str] = [config.start_hub]
        visited: set[str] = {config.start_hub}

        while queue:
            curr = queue.pop(0)
            if curr == config.end_hub:
                return

            for nxt in config.adj[curr]:
                if nxt not in visited:
                    if config.zones[nxt].zone_type == ZoneType.BLOCKED:
                        continue
                    visited.add(nxt)
                    queue.append(nxt)

        raise ValueError(
            f"No valid path exists from '{config.start_hub}' to "
            f"'{config.end_hub}'"
        )


class PathFinder:
    """Space-Time A* Pathfinding algorithm for multi-drone scheduling."""

    def __init__(self, config: MapConfig) -> None:
        """Initialize PathFinder with map configuration."""
        self.config: MapConfig = config
        self.node_occ: dict[tuple[str, int], int] = defaultdict(int)
        self.edge_occ: dict[tuple[tuple[str, str], int], int] = defaultdict(
            int
        )
        self._dist_to_end: dict[str, float] = self._compute_base_distances()

    def _compute_base_distances(self) -> dict[str, float]:
        """Compute shortest path distance from all nodes to end_hub."""
        dist: dict[str, float] = {self.config.end_hub: 0.0}
        pq: list[tuple[float, str]] = [(0.0, self.config.end_hub)]

        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            for v in self.config.adj[u]:
                v_zone = self.config.zones[v]
                if v_zone.zone_type == ZoneType.BLOCKED:
                    continue
                weight = self.config.zones[u].zone_type.cost
                if d + weight < dist.get(v, float("inf")):
                    dist[v] = d + weight
                    heapq.heappush(pq, (dist[v], v))
        return dist

    def _heuristic(self, zone_name: str) -> float:
        """Estimate remaining cost to end_hub ignoring congestion."""
        return self._dist_to_end.get(zone_name, 0.0)

    def find_schedule(self) -> dict[int, list[tuple[int, str]]]:
        """Schedule space-time paths for all drones."""
        schedules: dict[int, list[tuple[int, str]]] = {}

        for drone_id in range(1, self.config.nb_drones + 1):
            path = self._find_drone_path()
            if not path:
                raise ValueError(f"Could not route Drone D{drone_id}")
            self._reserve_path(path)
            schedules[drone_id] = path

        return schedules

    def _find_drone_path(self) -> list[tuple[int, str]]:
        """Find optimal space-time path for a single drone using A*."""
        start = self.config.start_hub
        end = self.config.end_hub

        priority_visits = int(
            self.config.zones[start].zone_type == ZoneType.PRIORITY
        )
        sequence = count()
        pq: list[
            tuple[float, int, int, int, str, list[tuple[int, str]]]
        ] = [
            (
                self._heuristic(start),
                0,
                -priority_visits,
                next(sequence),
                start,
                [(0, start)],
            )
        ]
        best_priority_visits: dict[tuple[str, int], int] = {}

        while pq:
            _, t, neg_priority_visits, _, curr, path = heapq.heappop(pq)
            priority_visits = -neg_priority_visits

            state_key = (curr, t)
            if priority_visits <= best_priority_visits.get(state_key, -1):
                continue
            best_priority_visits[state_key] = priority_visits

            if curr == end:
                return path

            curr_zone = self.config.zones[curr]
            if (
                curr == start
                or self.node_occ[(curr, t + 1)] < curr_zone.max_drones
            ):
                heapq.heappush(
                    pq,
                    (
                        t + 1 + self._heuristic(curr),
                        t + 1,
                        -priority_visits,
                        next(sequence),
                        curr,
                        path + [(t + 1, curr)],
                    ),
                )

            for nxt in self.config.adj[curr]:
                nxt_zone = self.config.zones[nxt]
                if nxt_zone.zone_type == ZoneType.BLOCKED:
                    continue

                pair = sorted([curr, nxt])
                norm_edge = (pair[0], pair[1])
                conn = self.config.conn_map[norm_edge]

                if self.edge_occ[(norm_edge, t)] >= conn.max_link_capacity:
                    continue

                if nxt_zone.zone_type in (
                    ZoneType.NORMAL,
                    ZoneType.PRIORITY,
                ):
                    arr_time = t + 1
                    next_priority_visits = priority_visits + int(
                        nxt_zone.zone_type == ZoneType.PRIORITY
                    )
                    if (
                        nxt == end
                        or self.node_occ[(nxt, arr_time)]
                        < nxt_zone.max_drones
                    ):
                        heapq.heappush(
                            pq,
                            (
                                arr_time + self._heuristic(nxt),
                                arr_time,
                                -next_priority_visits,
                                next(sequence),
                                nxt,
                                path + [(arr_time, nxt)],
                            ),
                        )

                elif nxt_zone.zone_type == ZoneType.RESTRICTED:
                    if (
                        self.edge_occ[(norm_edge, t + 1)]
                        >= conn.max_link_capacity
                    ):
                        continue
                    arr_time = t + 2
                    if (
                        nxt == end
                        or self.node_occ[(nxt, arr_time)]
                        < nxt_zone.max_drones
                    ):
                        conn_label = f"{curr}-{nxt}"
                        heapq.heappush(
                            pq,
                            (
                                arr_time + self._heuristic(nxt),
                                arr_time,
                                -priority_visits,
                                next(sequence),
                                nxt,
                                path + [
                                    (t + 1, conn_label),
                                    (arr_time, nxt),
                                ],
                            ),
                        )

        return []

    def _reserve_path(self, path: list[tuple[int, str]]) -> None:
        """Reserve node and edge capacity along drone's planned path."""
        for i in range(len(path) - 1):
            t_curr, loc_curr = path[i]
            t_next, loc_next = path[i + 1]

            if loc_curr == loc_next:
                self.node_occ[(loc_curr, t_next)] += 1
            elif "-" in loc_next and loc_next not in self.config.zones:
                u, v = loc_next.split("-", 1)
                pair = sorted([u, v])
                norm_edge = (pair[0], pair[1])
                self.edge_occ[(norm_edge, t_curr)] += 1
            else:
                if loc_curr in self.config.zones:
                    pair = sorted([loc_curr, loc_next])
                else:
                    u, v = loc_curr.split("-", 1)
                    pair = sorted([u, v])
                norm_edge = (pair[0], pair[1])
                self.edge_occ[(norm_edge, t_curr)] += 1

                self.node_occ[(loc_next, t_next)] += 1


class Visualizer:
    """Provides colored terminal output for visual simulation feedback."""

    COLOR_MAP = {
        "green": "\033[92m",
        "yellow": "\033[93m",
        "blue": "\033[94m",
        "red": "\033[91m",
        "gray": "\033[90m",
        "cyan": "\033[96m",
        "magenta": "\033[95m",
        "reset": "\033[0m",
    }

    TYPE_DEFAULT_COLORS = {
        ZoneType.NORMAL: "reset",
        ZoneType.PRIORITY: "green",
        ZoneType.RESTRICTED: "red",
        ZoneType.BLOCKED: "gray",
    }

    @classmethod
    def format_move(
        cls, drone_id: int, target: str, color_name: str
    ) -> str:
        """Format movement output with ANSI coloring."""
        color_code = cls.COLOR_MAP.get(color_name.lower(), "")
        reset_code = cls.COLOR_MAP["reset"] if color_code else ""
        return f"{color_code}D{drone_id}-{target}{reset_code}"


class SimulationEngine:
    """Manages turn-by-turn simulation execution and output."""

    def __init__(self, config: MapConfig) -> None:
        """Initialize SimulationEngine with MapConfig."""
        self.config: MapConfig = config
        self.path_finder: PathFinder = PathFinder(config)

    def run(self, show_capacity: bool = False) -> None:
        """Execute the drone simulation and print turn movements."""
        schedules = self.path_finder.find_schedule()

        max_turns = 0
        for path in schedules.values():
            if path:
                max_turns = max(max_turns, path[-1][0])

        drone_positions: dict[int, str] = {
            d: self.config.start_hub
            for d in range(1, self.config.nb_drones + 1)
        }

        for turn in range(1, max_turns + 1):
            turn_moves: list[str] = []

            for drone_id in range(1, self.config.nb_drones + 1):
                path = schedules[drone_id]
                loc_at_turn = None
                for t, loc in path:
                    if t == turn:
                        loc_at_turn = loc
                        break

                if loc_at_turn is not None:
                    prev_loc = drone_positions[drone_id]
                    if loc_at_turn != prev_loc:
                        drone_positions[drone_id] = loc_at_turn

                        if loc_at_turn in self.config.zones:
                            z = self.config.zones[loc_at_turn]
                            zone_color = (
                                z.color
                                if z.color != "none"
                                else Visualizer.TYPE_DEFAULT_COLORS.get(
                                    z.zone_type, "reset"
                                )
                            )
                        elif "-" in loc_at_turn:
                            zone_color = "yellow"
                        else:
                            zone_color = "reset"

                        formatted_move = Visualizer.format_move(
                            drone_id, loc_at_turn, zone_color
                        )
                        turn_moves.append(formatted_move)

            if turn_moves:
                print(" ".join(turn_moves))

            if show_capacity:
                self._print_capacity_info(drone_positions)

    def _print_capacity_info(
        self, drone_positions: dict[int, str]
    ) -> None:
        """Print capacity usage for zones and connections."""
        z_counts: dict[str, int] = defaultdict(int)
        c_counts: dict[tuple[str, str], int] = defaultdict(int)

        for pos in drone_positions.values():
            if pos in self.config.zones:
                z_counts[pos] += 1
            elif "-" in pos:
                u, v = pos.split("-", 1)
                norm_edge = (min(u, v), max(u, v))
                c_counts[norm_edge] += 1

        for z_name, z_obj in sorted(self.config.zones.items()):
            curr = z_counts[z_name]
            print(f"Zone {z_name}: {curr}/{z_obj.max_drones} drones")

        for conn in self.config.connections:
            curr = c_counts[conn.key]
            print(
                f"Connection {conn.zone1}-{conn.zone2}: "
                f"{curr}/{conn.max_link_capacity} capacity used"
            )


def main() -> None:
    """Main entry point for command-line execution."""
    parser = argparse.ArgumentParser(
        description="Fly-in Drone Routing Simulation System"
    )
    parser.add_argument(
        "map_file",
        type=str,
        help="Path to the map configuration file (.txt)",
    )

    args = parser.parse_args()
    filepath = args.map_file.strip()
    map_parser = MapParser()

    try:
        config = map_parser.parse_file(filepath)
        engine = SimulationEngine(config)
        engine.run()
    except ValueError as e:
        sys.stderr.write(f"Parsing Error: {e}\n")
        sys.exit(1)
    except FileNotFoundError:
        sys.stderr.write(f"Error: Map file '{filepath}' not found\n")
        sys.exit(1)
    except PermissionError:
        sys.stderr.write(
            f"Error: Permission denied reading '{filepath}'\n"
        )
        sys.exit(1)
    except Exception as e:
        sys.stderr.write(f"Unexpected Error: {e}\n")
        sys.exit(1)


if __name__ == "__main__":
    main()
