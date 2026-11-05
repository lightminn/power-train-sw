"""Isolation gate comparison (spec §8.2): endpoints, not names, are compared."""
import terrain_map_isolation_gate as G


def snap(*pubs, nodes=("/terrain_map",)):
    return {"publishers": sorted([list(p) for p in pubs]), "nodes": list(nodes)}


W = ("/wheel_states", "powertrain_msgs/msg/WheelStates", "/chassis")


def test_a_second_publisher_with_the_same_name_fails():
    ok, ev = G.compare(snap(W + ("gid-a",)), snap(W + ("gid-a",), W + ("gid-b",)))
    assert not ok and ev["added_publishers"] == [list(W + ("gid-b",))]


def test_a_vanished_twin_fails():
    ok, _ = G.compare(snap(W + ("gid-a",), W + ("gid-b",)), snap(W + ("gid-a",)))
    assert not ok


def test_infrastructure_topics_and_unchanged_graphs_pass():
    rosout = ("/rosout", "rcl_interfaces/msg/Log", "/terrain_map", "gid-x")
    ok, _ = G.compare(snap(W + ("gid-a",)), snap(W + ("gid-a",), rosout))
    assert ok
