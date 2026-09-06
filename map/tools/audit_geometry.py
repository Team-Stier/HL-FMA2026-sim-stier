"""Audit source coverage, cell geometry and native routing; findings require interpretation."""
import argparse
import collections
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import lanelet2.io as io
import lanelet2.routing as routing
import lanelet2.traffic_rules as rules
import networkx as nx
import numpy as np
import shapely
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union
from shapely.validation import explain_validity


def provenance(paths):
    inputs = {}
    for path in paths:
        with path.open("rb") as stream:
            inputs[str(path.resolve())] = hashlib.file_digest(stream, "sha256").hexdigest()
    return {"inputs": inputs, "python": sys.version, "lanelet2_io": io.__file__,
            "versions": {"numpy": np.__version__, "shapely": shapely.__version__,
                         "networkx": nx.__version__}}


def save_result(result, output, baseline=None):
    if baseline:
        expected = json.loads(baseline.read_text())
        actual = json.loads(json.dumps(result))
        expected.pop("provenance", None)
        actual.pop("provenance", None)
        differences = [key for key in expected.keys() | actual.keys()
                       if expected.get(key) != actual.get(key)]
        assert not differences, f"Baseline mismatch: {sorted(differences)}"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


def audit(map_dir, xodr):
    r = json.loads((map_dir / 'build_report.json').read_text())
    metadata = provenance([map_dir / 'hdmap.bin', map_dir / 'build_report.json', xodr, Path(__file__)])
    assert metadata['inputs'][str((map_dir / 'hdmap.bin').resolve())] == r['binary_sha256'], 'Map and build_report.json do not match'
    x = ET.parse(xodr).getroot()
    roads = {int(d.get('id')): d for d in x.findall('road')}
    m = io.load(str(map_dir / 'hdmap.bin'), io.Origin(0, 0))
    lanes = {l.id: l for l in m.laneletLayer}
    xy = lambda p: (p.x, p.y)
    poly = lambda p: Polygon([xy(i) for i in p])
    shapes = {i: poly(l.polygon2d()) for i, l in lanes.items()}
    centers = {i: LineString([xy(p) for p in l.centerline]) for i, l in lanes.items()}
    output = {'source_roads': len(roads), 'generated_roads': len(set((v['xodr'][0] for v in r['lanelets'].values()))), 'lanelets': len(lanes), 'source_coverage_gaps': [], 'non_driving_generated': [], 'invalid_lanelets': [], 'invalid_cells': [], 'missing_cells': [], 'cell_union_errors': [], 'cell_stopline_separation': [], 'bad_previous_geometry': [], 'stopline_orientation_errors': [], 'cell_id_errors': []}
    for i, p in shapes.items():
        if not p.is_valid or p.area < 1e-09:
            output['invalid_lanelets'].append([i, p.area, explain_validity(p)])
    gen = collections.defaultdict(list)
    for i, l in r['lanelets'].items():
        gen[tuple(l['xodr'])].append((l['s'][0], l['s'][1], int(i)))
    for rid, road in roads.items():
        secs = road.findall('lanes/laneSection')
        for n, s in enumerate(secs):
            begin = float(s.get('s'))
            end = float(secs[n + 1].get('s')) if n + 1 < len(secs) else float(road.get('length'))
            for l in s.findall('./*/lane'):
                lid = int(l.get('id'))
                typ = l.get('type')
                if lid == 0:
                    continue
                intervals = sorted(gen.get((rid, n, lid), []))
                if typ != 'driving':
                    if intervals:
                        output['non_driving_generated'].append([rid, n, lid, typ, intervals])
                    continue
                last = begin
                for a, b, i in intervals:
                    if abs(a - last) > 1e-06:
                        output['source_coverage_gaps'].append([rid, n, lid, last, a])
                    last = b
                if abs(last - end) > 1e-06:
                    output['source_coverage_gaps'].append([rid, n, lid, last, end])
    cells = {int(c.attributes['cell_id']): c for c in m.polygonLayer if 'type' in c.attributes and c.attributes['type'] == 'hdmap_cell'}
    cell_shapes = {i: poly(c) for i, c in cells.items()}
    by_parent = collections.defaultdict(list)
    stops = {s.id: s for s in m.lineStringLayer if 'type' in s.attributes and s.attributes['type'] == 'stop_line'}
    stop_shapes = {i: LineString([xy(p) for p in s]) for i, s in stops.items()}
    stop_cells = collections.defaultdict(list)
    for cid, c in cells.items():
        p = cell_shapes[cid]
        attrs = c.attributes
        pid = int(attrs['parent_lanelet_id'])
        by_parent[pid].append(cid)
        if not p.is_valid or p.area < 1e-10:
            output['invalid_cells'].append([cid, p.area, explain_validity(p)])
        if 'previous_cell_id' in attrs:
            prev = int(attrs['previous_cell_id'])
            gap = p.distance(cell_shapes[prev])
            if gap > 1e-05:
                output['bad_previous_geometry'].append([cid, prev, gap, pid])
        if attrs['stopline_ids']:
            for sid in map(int, attrs['stopline_ids'].split(',')):
                stop_cells[sid].append(cid)
                gap = p.distance(stop_shapes[sid])
                if gap > 0.02:
                    output['cell_stopline_separation'].append({'cell': cid, 'lanelet': pid, 'xodr': r['lanelets'][str(pid)]['xodr'], 'stopline': sid, 'distance_m': gap, 'source': dict(stops[sid].attributes)})
    max_diff = (0, None)
    max_overlap = (0, None)
    for pid, p in shapes.items():
        cs = by_parent[pid]
        if not cs:
            output['missing_cells'].append(pid)
            continue
        u = unary_union([cell_shapes[c] for c in cs])
        diff = p.symmetric_difference(u).area
        overlap = sum((cell_shapes[c].area for c in cs)) - u.area
        if diff > max_diff[0]:
            max_diff = (diff, pid)
        if overlap > max_overlap[0]:
            max_overlap = (overlap, pid)
        if diff > 1e-05 or overlap > 1e-05:
            output['cell_union_errors'].append([pid, diff, overlap])
        orders = sorted(((int(cells[c].attributes['index_in_lanelet']), c) for c in cs))
        if [i for i, c in orders] != list(range(len(cs))):
            output['cell_id_errors'].append([pid, orders])
    output['cells'] = len(cells)
    output['maximum_cell_parent_symmetric_difference_m2'] = max_diff
    output['maximum_sibling_cell_overlap_m2'] = max_overlap
    output['cell_longitudinal_length_m'] = {'min': min((float(c.attributes['centerline_length_m']) for c in cells.values())), 'max': max((float(c.attributes['centerline_length_m']) for c in cells.values()))}
    output['stoplines_without_cells'] = [{'id': sid, 'attributes': dict(stops[sid].attributes), 'center': list(stop_shapes[sid].centroid.coords)[0]} for sid in stops if sid not in stop_cells]
    tr = rules.create(rules.Locations.Germany, rules.Participants.Vehicle)
    rg = routing.RoutingGraph(m, tr)
    g = nx.DiGraph()
    g.add_nodes_from(lanes)
    forward = nx.DiGraph()
    forward.add_nodes_from(lanes)
    for lid, l in lanes.items():
        for target in rg.following(l, False):
            forward.add_edge(lid, target.id)
        for target in rg.following(l, True):
            g.add_edge(lid, target.id)
    output['native_routing_errors'] = list(rg.checkValidity(False))
    explicit = set(map(tuple, r['routing_links']))
    native = set(forward.edges)
    output['explicit_edges_missing_native'] = sorted(explicit - native)
    output['native_forward_edges_not_explicit'] = sorted(native - explicit)
    output['graph_stats'] = {'edges_following_only': forward.number_of_edges(), 'edges_with_lane_changes': g.number_of_edges(), 'weak_components_following_only': sorted(map(len, nx.weakly_connected_components(forward)), reverse=True), 'weak_components_with_lane_changes': sorted(map(len, nx.weakly_connected_components(g)), reverse=True), 'strong_components_with_lane_changes': sorted(map(len, nx.strongly_connected_components(g)), reverse=True)}

    def description(i):
        return {'lanelet': i, 'xodr': r['lanelets'][str(i)]['xodr'], 's': r['lanelets'][str(i)]['s'], 'begin': list(centers[i].coords)[0], 'end': list(centers[i].coords)[-1]}
    output['isolated_weak_components_with_lane_changes'] = [[description(i) for i in sorted(c)] for c in sorted(nx.weakly_connected_components(g), key=len, reverse=True)[1:]]
    output['dead_ends_with_lane_changes'] = [description(i) for i, d in g.out_degree if d == 0]
    output['unreachable_starts_with_lane_changes'] = [description(i) for i, d in g.in_degree if d == 0]
    for st in r['stoplines']:
        sid = st['id']
        s = stops[sid]
        line = stop_shapes[sid]
        vec = np.array(line.coords[-1]) - np.array(line.coords[0])
        vec = vec / np.linalg.norm(vec)
        for pid in st['lanelets']:
            cl = centers[pid]
            d = cl.project(line.centroid)
            a = np.array(cl.interpolate(max(0, d - 0.1)).coords[0])
            b = np.array(cl.interpolate(min(cl.length, d + 0.1)).coords[0])
            v = b - a
            alignment = abs(np.dot(vec, v) / np.linalg.norm(v))
            if alignment > 0.2:
                output['stopline_orientation_errors'].append([sid, pid, float(alignment), r['lanelets'][str(pid)]['xodr']])
    output['source_roadmark_lanechange_counts'] = {str(k): v for k, v in collections.Counter(((e.get('type'), e.get('laneChange')) for e in x.findall('.//roadMark'))).items()}
    output['summary_counts'] = {k: len(v) for k, v in output.items() if isinstance(v, list)}
    output['provenance'] = metadata
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, default=Path("map"))
    parser.add_argument("--xodr", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, help="Assert all findings equal this earlier audit JSON")
    args = parser.parse_args()
    result = audit(args.map_dir, args.xodr)
    save_result(result, args.output, args.baseline)
    print(json.dumps(result["summary_counts"], indent=2))
