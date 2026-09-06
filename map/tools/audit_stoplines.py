"""Compare every stopline with OSGB rectangle candidates; geometric proximity is not proof of applicability."""
import argparse
import collections
import json
from pathlib import Path

import lanelet2.io as io
import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

from audit_geometry import provenance, save_result


def audit(map_dir, mesh_path):
    m = io.load(str(map_dir / 'hdmap.bin'), io.Origin(0, 0))
    report = json.loads((map_dir / 'build_report.json').read_text())
    mesh = json.loads(mesh_path.read_text())
    metadata = provenance([map_dir / 'hdmap.bin', map_dir / 'build_report.json', mesh_path, Path(__file__), Path(__file__).with_name('audit_geometry.py')])
    assert metadata['inputs'][str((map_dir / 'hdmap.bin').resolve())] == report['binary_sha256'], 'Map and build_report.json do not match'
    stops = [s for s in m.lineStringLayer if 'type' in s.attributes and s.attributes['type'] == 'stop_line']
    stops.sort(key=lambda s: s.id)
    centers = np.array([s['center'] for s in mesh])
    vectors = []
    lengths = []
    for mark in mesh:
        p = np.array(mark['corners'])
        v = max([p[1] - p[0], p[3] - p[0]], key=lambda v: np.linalg.norm(v[:2]))
        lengths.append(np.linalg.norm(v[:2]))
        vectors.append(v[:2] / np.linalg.norm(v[:2]))
    vectors = np.array(vectors)
    lengths = np.array(lengths)
    rows = []
    source = {s['id']: s for s in report['stoplines']}
    # ponytail: full stop-by-mesh scan; add spatial filtering if candidate volume grows.
    for s in stops:
        p = np.array([(p.x, p.y, p.z) for p in s])
        center = p.mean(axis=0)
        v = p[-1, :2] - p[0, :2]
        length = np.linalg.norm(v)
        v /= length
        dxy = np.linalg.norm(centers[:, :2] - center[:2], axis=1)
        dz = np.abs(centers[:, 2] - center[2])
        angle = np.degrees(np.arccos(np.clip(np.abs(vectors @ v), 0, 1)))
        dlen = np.abs(lengths - length)

        def desc(i):
            delta = centers[i, :2] - center[:2]
            return {'candidate': int(i), 'xy_distance_m': float(dxy[i]), 'height_difference_m': float(dz[i]), 'angle_difference_deg': float(angle[i]), 'length_difference_m': float(dlen[i]), 'center': centers[i].tolist(), 'along_stopline_m': float(delta @ v), 'normal_to_stopline_m': float(abs(v[0] * delta[1] - v[1] * delta[0]))}
        eligible = np.where((dz < 0.6) & (angle < 10) & (dlen < 0.8))[0]
        nearest = int(np.argmin(dxy))
        compatible = int(eligible[np.argmin(dxy[eligible])]) if len(eligible) else None
        rows.append({'stopline': s.id, 'attributes': dict(s.attributes), 'lanelets': source[s.id]['lanelets'], 'center': center.tolist(), 'length_m': float(length), 'nearest_mesh': desc(nearest), 'compatible_mesh': desc(compatible) if compatible is not None else None})
    assigned = collections.defaultdict(list)
    for row in rows:
        c = row['compatible_mesh']
        if c and c['xy_distance_m'] < 1:
            assigned[c['candidate']].append(row['stopline'])
    stopcenters = np.array([row['center'] for row in rows])
    lanes = list(m.laneletLayer)
    lp = [Polygon([(p.x, p.y) for p in l.polygon2d()]) for l in lanes]
    ltree = STRtree(lp)
    lcenters = [LineString([(p.x, p.y) for p in l.centerline]) for l in lanes]
    unmatched = []
    for i, mark in enumerate(mesh):
        if i in assigned:
            continue
        dxy = np.linalg.norm(stopcenters[:, :2] - centers[i, :2], axis=1)
        j = int(np.argmin(dxy))
        p = Point(centers[i, :2])
        near = []
        for k in ltree.query(p.buffer(0.7)):
            if lp[k].distance(p) > 0.7:
                continue
            cl = lcenters[k]
            d = cl.project(p)
            a = np.array(cl.interpolate(max(0, d - 0.1)).coords[0])
            b = np.array(cl.interpolate(min(cl.length, d + 0.1)).coords[0])
            v = b - a
            cos = abs(v @ vectors[i]) / np.linalg.norm(v)
            if cos > 0.2:
                continue
            near.append({'lanelet': lanes[k].id, 'xodr': report['lanelets'][str(lanes[k].id)]['xodr'], 'centerline_s_m': d, 'lanelet_length_m': cl.length, 'distance_to_lane_m': lp[k].distance(p)})
        unmatched.append({'candidate': i, 'center': mark['center'], 'length_m': float(lengths[i]), 'nearest_stopline': rows[j]['stopline'], 'nearest_stopline_distance_m': float(dxy[j]), 'candidate_lanes': near})
    out = {'counts': {'stoplines': len(rows), 'mesh_rectangles': len(mesh), 'compatible_near_0_5m': sum((row['compatible_mesh'] is not None and row['compatible_mesh']['xy_distance_m'] < 0.5 for row in rows)), 'compatible_near_1m': sum((row['compatible_mesh'] is not None and row['compatible_mesh']['xy_distance_m'] < 1 for row in rows)), 'compatible_over_1m': sum((row['compatible_mesh'] is None or row['compatible_mesh']['xy_distance_m'] >= 1 for row in rows)), 'unmatched_mesh_candidates': len(unmatched)}, 'all_stopline_mesh_matches': rows, 'stops_without_near_compatible_mesh': [row for row in rows if row['compatible_mesh'] is None or row['compatible_mesh']['xy_distance_m'] >= 1], 'mesh_candidates_without_near_stopline': unmatched, 'mesh_assignment_collisions': {str(i): ids for i, ids in assigned.items() if len(ids) > 1}}
    out['provenance'] = metadata
    return out


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, default=Path("map"))
    parser.add_argument("--mesh", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, help="Assert all findings equal this earlier audit JSON")
    args = parser.parse_args()
    result = audit(args.map_dir, args.mesh)
    save_result(result, args.output, args.baseline)
    print(json.dumps(result["counts"], indent=2))
