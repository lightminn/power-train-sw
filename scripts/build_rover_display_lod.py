#!/usr/bin/env python3
"""Build bounded display meshes from preserved CAD references (offline only).

Requires numpy, scipy and fast-simplification; the GUI needs none of them.
"""
import hashlib
import json
from pathlib import Path
import struct

import fast_simplification
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def weld_display(vertices, faces, grid_mm):
    """Merge submillimetre STL seams before reducing their disconnected faces."""
    cells = np.rint(vertices / grid_mm).astype(np.int32)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    weights = np.bincount(inverse)
    points = np.column_stack([np.bincount(inverse, weights=vertices[:, axis]) / weights for axis in range(3)])
    triangles = inverse[faces]
    triangles = triangles[(triangles[:, 0] != triangles[:, 1]) & (triangles[:, 1] != triangles[:, 2]) & (triangles[:, 0] != triangles[:, 2])]
    _, unique = np.unique(np.sort(triangles, axis=1), axis=0, return_index=True)
    return points, triangles[np.sort(unique)]


def simplify_components(vertices, faces, target):
    # Simplifying the whole CAD preferentially destroys small disconnected
    # housings/shafts. Allocate each visible solid its own display budget.
    rows = np.repeat(faces[:, 0], 2)
    cols = faces[:, 1:].reshape(-1)
    graph = coo_matrix((np.ones(len(rows), dtype=np.uint8), (rows, cols)), shape=(len(vertices), len(vertices))).tocsr()
    _, labels = connected_components(graph, directed=False)
    groups = labels[faces[:, 0]]
    order = np.argsort(groups)
    solids = []
    for indices in np.split(order, np.flatnonzero(np.diff(groups[order])) + 1):
        ids, inverse = np.unique(faces[indices], return_inverse=True)
        points = vertices[ids]
        span = np.ptp(points, axis=0)
        if span.max() < 6 or len(indices) < 4:
            continue
        local = inverse.reshape(-1, 3)
        area = np.linalg.norm(np.cross(points[local[:, 1]] - points[local[:, 0]], points[local[:, 2]] - points[local[:, 0]]), axis=1).sum() / 2
        solids.append((points, local, area ** .75))
    total = sum(s[2] for s in solids)
    output_points, output_faces, offset = [], [], 0
    for points, triangles, weight in solids:
        source_bounds = np.array([points.min(0), points.max(0)])
        points, triangles = weld_display(points, triangles, 1.0)
        if len(triangles) == 0:
            continue
        original_triangles = triangles
        budget = min(len(triangles), max(32, round(target * weight / total)))
        reduced, triangles = fast_simplification.simplify(points, triangles.astype(np.int32), target_count=budget, agg=9)
        if len(reduced) == 0 or len(triangles) == 0:
            reduced, triangles = points, original_triangles
        # Keep a solid at higher resolution when its protrusion is erased.
        error = np.max(np.abs(np.array([reduced.min(0), reduced.max(0)]) - source_bounds))
        if error > 4:
            reduced, triangles = fast_simplification.simplify(points, original_triangles.astype(np.int32), target_count=min(len(original_triangles), budget * 4), agg=9)
            if len(reduced) == 0 or len(triangles) == 0:
                reduced, triangles = points, original_triangles
        output_points.append(reduced); output_faces.append(triangles + offset); offset += len(reduced)
    print("visible solids", len(solids), flush=True)
    return np.concatenate(output_points), np.concatenate(output_faces)


def main():
    root = Path(__file__).resolve().parents[1] / "operator_console/assets/rover3d"
    output = root / "display-meshes"
    output.mkdir(exist_ok=True)
    model = json.loads((root / "model.json").read_text())
    manifest = {"method": "0.05 mm seam welding; separate visible solids >= 6 mm; 1 mm display clustering and quadric-error LOD; original URDF frames and CAD references retained", "meshes": []}
    for link in model["links"]:
        name = link["name"]
        if name.startswith(("arm", "tool", "finger", "pinion")):
            continue
        for visual in link["visuals"]:
            filename = visual["mesh"]
            source = (root / "meshes" / filename).read_bytes()
            vertex_count, face_count = struct.unpack_from("<II", source)
            vertices = np.frombuffer(source, dtype="<f4", count=vertex_count * 3, offset=8).reshape(-1, 3)
            faces = np.frombuffer(source, dtype="<u4", count=face_count * 3, offset=8 + vertex_count * 12).reshape(-1, 3)
            target = (30000 if name == "base_link" else 12000 if "chassis_pivot" in name
                      else 8000 if "bogie_pivot" in name else 4000 if "kingpin" in name
                      else 5000 if "bl70200s" in name else 1500 if "bar_differential" in name else 750)
            cache = Path('/dev/shm/rover-lod-weld')
            cache.mkdir(exist_ok=True)
            cached = cache / (hashlib.sha256(source).hexdigest() + '.npz')
            if cached.exists():
                with np.load(cached) as saved:
                    points, triangles = saved['points'], saved['triangles']
            else:
                points, triangles = weld_display(vertices, faces, .05)
                np.savez(cached, points=points, triangles=triangles)
            points, triangles = simplify_components(points, triangles, target)
            assert np.isfinite(points).all() and triangles.min() >= 0 and triangles.max() < len(points)
            bounds_error = float(np.max(np.abs(np.array([points.min(0), points.max(0)]) - np.array([vertices.min(0), vertices.max(0)]))))
            # Display-only LOD: tiny protrusions can be lost, not the main body.
            assert bounds_error < 20, (name, bounds_error)
            data = struct.pack("<II", len(points), len(triangles)) + points.astype("<f4").tobytes() + triangles.astype("<u4").tobytes()
            (output / filename).write_bytes(data)
            row = dict(link=name, mesh=filename, source_sha256=hashlib.sha256(source).hexdigest(), source_triangles=face_count, triangles=len(triangles), bytes=len(data), bounds_error_mm=bounds_error)
            manifest["meshes"].append(row)
            print(name, face_count, "->", len(triangles), "bounds mm", round(bounds_error, 3), flush=True)
    manifest["triangles"] = sum(row["triangles"] for row in manifest["meshes"])
    manifest["bytes"] = sum(row["bytes"] for row in manifest["meshes"])
    assert manifest["triangles"] < 300000 and manifest["bytes"] < 8000000
    (root / "display-mesh-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("TOTAL", manifest["triangles"], manifest["bytes"], flush=True)


if __name__ == "__main__":
    main()
