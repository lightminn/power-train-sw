"""Deployment isolation gate for the terrain map service (spec §8.2).

The map node may only subscribe.  Take a graph snapshot before the service
starts and one after, then compare: no new publisher on any topic (ROS's own
/rosout and /parameter_events excepted), nothing new on /tf or /tf_static, and
the /terrain_map node really is running in the second snapshot.

    python3 terrain_map_isolation_gate.py snapshot before.json
    (start the service — e.g. docker compose ... --profile terrain-map up -d powertrain_terrain_map)
    python3 terrain_map_isolation_gate.py snapshot after.json
    python3 terrain_map_isolation_gate.py compare before.json after.json
"""
from __future__ import annotations

import argparse
import json
import sys
import time

INFRA_TOPICS = {"/rosout", "/parameter_events"}
SNAPSHOT_NODE = "terrain_map_isolation_gate"


def snapshot(path: str, settle_s: float) -> dict:
    import rclpy
    rclpy.init()
    node = rclpy.create_node(SNAPSHOT_NODE)
    try:
        time.sleep(settle_s)                                # discovery
        rclpy.spin_once(node, timeout_sec=0.1)
        publishers = []
        for topic, types in node.get_topic_names_and_types():
            for info in node.get_publishers_info_by_topic(topic):
                if info.node_name == SNAPSHOT_NODE:
                    continue
                publishers.append([topic, ",".join(types), f"{info.node_namespace.rstrip('/')}/{info.node_name}"])
        nodes = sorted(f"{ns.rstrip('/')}/{name}" for name, ns in node.get_node_names_and_namespaces()
                       if name != SNAPSHOT_NODE)
        result = {"publishers": sorted(publishers), "nodes": nodes}
    finally:
        node.destroy_node()
        rclpy.shutdown()
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=1)
    return result


def compare(before: dict, after: dict) -> tuple[bool, dict]:
    added = [p for p in after["publishers"] if p not in before["publishers"] and p[0] not in INFRA_TOPICS]
    removed = [p for p in before["publishers"] if p not in after["publishers"] and p[0] not in INFRA_TOPICS]
    tf = [p for p in added if p[0] in ("/tf", "/tf_static")]
    map_running = "/terrain_map" in after["nodes"]
    evidence = {"map_node_running": map_running, "added_publishers": added, "removed_publishers": removed,
                "new_tf_publishers": tf, "publishers_before": len(before["publishers"]),
                "publishers_after": len(after["publishers"])}
    return map_running and not added and not removed, evidence


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    snap = sub.add_parser("snapshot")
    snap.add_argument("out")
    snap.add_argument("--settle-s", type=float, default=3.0)
    comp = sub.add_parser("compare")
    comp.add_argument("before")
    comp.add_argument("after")
    args = parser.parse_args(argv)
    if args.cmd == "snapshot":
        result = snapshot(args.out, args.settle_s)
        print(json.dumps({"publishers": len(result["publishers"]), "nodes": result["nodes"]}))
        return 0
    with open(args.before, encoding="utf-8") as fh:
        before = json.load(fh)
    with open(args.after, encoding="utf-8") as fh:
        after = json.load(fh)
    ok, evidence = compare(before, after)
    print(json.dumps(dict(evidence, verdict="PASS" if ok else "FAIL")))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
