"""Small routing cases, capacity checks, and malformed map inputs."""

from collections import Counter
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

from main import MapConfig, MapParser, PathFinder

ROOT = Path(__file__).resolve().parents[1]
BASE = ("nb_drones: 1\nstart_hub: start 0 0\n"
        "end_hub: end 2 0\nconnection: start-end\n")


class RoutingTests(unittest.TestCase):
    def setUp(self) -> None:
        workspace = tempfile.TemporaryDirectory(prefix="flyin-test-")
        self.addCleanup(workspace.cleanup)
        self.work = Path(workspace.name)

    def parse(self, text: str) -> MapConfig:
        path = self.work / "map.txt"
        path.write_text(text, encoding="utf-8")
        return MapParser().parse_file(str(path))

    def check_schedule(self, text: str) -> dict[int, list[tuple[int, str]]]:
        config = self.parse(text)
        schedules = PathFinder(config).find_schedule()
        self.assertEqual(set(schedules), set(range(1, config.nb_drones + 1)))
        nodes: Counter[tuple[int, str]] = Counter()
        edges: Counter[tuple[int, tuple[str, ...]]] = Counter()
        for path in schedules.values():
            self.assertEqual(path[0], (0, config.start_hub))
            self.assertEqual(path[-1][1], config.end_hub)
            for index, (turn, location) in enumerate(path):
                self.assertEqual(turn, index)
                if location in config.zones:
                    self.assertNotEqual(
                        config.zones[location].zone_type.value, "blocked")
                    nodes[(turn, location)] += 1
                if index == 0:
                    continue
                previous = path[index - 1][1]
                if previous == location:
                    continue
                if location not in config.zones:
                    pair = tuple(sorted(location.split("-")))
                    self.assertEqual(previous, location.split("-")[0])
                    self.assertLess(index + 1, len(path))
                    self.assertEqual(path[index + 1][1],
                                     location.split("-")[1])
                elif previous not in config.zones:
                    pair = tuple(sorted(previous.split("-")))
                else:
                    pair = tuple(sorted((previous, location)))
                    self.assertIn(location, config.adj[previous])
                    self.assertNotEqual(
                        config.zones[location].zone_type.value, "restricted")
                self.assertEqual(len(pair), 2)
                self.assertIn((pair[0], pair[1]), config.conn_map)
                edges[(turn - 1, pair)] += 1
        for (_, name), occupancy in nodes.items():
            self.assertLessEqual(occupancy, config.zones[name].max_drones)
        for (_, pair), occupancy in edges.items():
            capacity = config.conn_map[(pair[0], pair[1])].max_link_capacity
            self.assertLessEqual(occupancy, capacity)
        return schedules

    def test_one_and_multiple_drones(self) -> None:
        for number in (1, 2, 5):
            with self.subTest(drones=number):
                paths = self.check_schedule(BASE.replace(
                    "nb_drones: 1", f"nb_drones: {number}"))
                self.assertEqual(max(p[-1][0] for p in paths.values()), number)

    def test_restricted_zone_and_bottleneck(self) -> None:
        for kind in ("normal", "restricted", "priority"):
            with self.subTest(zone=kind):
                text = ("nb_drones: 4\nstart_hub: start 0 0\n"
                        "end_hub: end 2 0\n"
                        f"hub: middle 1 0 [zone={kind}]\n"
                        "connection: start-middle\nconnection: middle-end\n")
                paths = self.check_schedule(text)
                first_arrival = min(p[-1][0] for p in paths.values())
                expected = 3 if kind == "restricted" else 2
                self.assertEqual(first_arrival, expected)

    def test_blocked_zone_and_custom_capacity(self) -> None:
        text = ("nb_drones: 4\nstart_hub: start 0 0\nend_hub: end 2 0\n"
                "hub: closed 1 1 [zone=blocked]\n"
                "connection: start-closed\nconnection: closed-end\n"
                "connection: start-end [max_link_capacity=2]\n")
        paths = self.check_schedule(text)
        self.assertEqual(max(p[-1][0] for p in paths.values()), 2)

    def test_invalid_maps(self) -> None:
        cases = [
            "", BASE.replace("nb_drones: 1", "nb_drones: 0"),
            BASE.replace("nb_drones: 1", "nb_drones: -1"),
            BASE.replace("nb_drones: 1", "nb_drones: abc"),
            BASE.replace("start_hub: start 0 0\n", ""),
            BASE.replace("end_hub: end 2 0\n", ""),
            BASE.replace("start 0 0", "start x 0"),
            BASE.replace("end 2 0", "end 2 0 [zone=blocked]"),
            BASE.replace("end 2 0", "end 2 0 [zone=unknown]"),
            BASE.replace("start-end", "start-unknown"),
            BASE.replace("start-end", "start-start"),
            BASE + "connection: end-start\n",
            BASE.replace("connection: start-end\n", ""),
            BASE.replace("start-end", "start-end [max_link_capacity=0]"),
            BASE.replace("end 2 0", "end 2 0 [color=red"),
            BASE + "hub: start 3 0\n", BASE + "unknown: data\n",
        ]
        for text in cases:
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    self.parse(text)

    def test_cli_output_and_errors(self) -> None:
        path = self.work / "map.txt"
        path.write_text(BASE, encoding="utf-8")
        for args, valid in (([str(path)], True), ([], False),
                            ([str(self.work / "missing.txt")], False)):
            result = subprocess.run([sys.executable, str(ROOT / "main.py"),
                                     *args], capture_output=True, text=True,
                                    cwd=self.work, timeout=5)
            if valid:
                self.assertEqual(result.returncode, 0, result.stderr)
                output = re.sub(r"\x1b\[[0-9;]*m", "", result.stdout)
                self.assertEqual(output.strip(), "D1-end")
            else:
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("Traceback", result.stderr)
                self.assertTrue(result.stderr)


if __name__ == "__main__":
    unittest.main()
