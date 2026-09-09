"""Check every cell's static cap, XODR junction tag, and optional pre-change geometry."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import lanelet2.io as io
import lanelet2.traffic_rules as rules
from shapely.geometry import Polygon, shape

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('directory', type=Path)
p.add_argument('--xodr', type=Path, required=True)
p.add_argument('--school-mesh', type=Path, default=Path('map/school_zone_mesh.json'))
p.add_argument('--baseline', type=Path)
a = p.parse_args()
report = json.loads((a.directory/'build_report.json').read_text())
sha = hashlib.sha256((a.directory/'hdmap.bin').read_bytes()).hexdigest()
assert sha == report['binary_sha256']
model = io.load(str(a.directory/'hdmap.bin'), io.Origin(0, 0))
source = ET.parse(a.xodr).getroot()
junctions = {r.get('id') for r in source.findall('road') if r.get('junction', '-1') != '-1'}
assert junctions == {c.get('connectingRoad') for c in source.findall('junction/connection')}
red = shape(json.loads(a.school_mesh.read_text())['geometry'])
counts, children, mixed = Counter(), defaultdict(list), 0
for cell in model.polygonLayer:
    if 'type' not in cell.attributes or cell.attributes['type'] != 'hdmap_cell':
        continue
    polygon = Polygon([(v.x, v.y) for v in cell])
    overlap = polygon.intersection(red).area
    school = overlap > max(1e-9, polygon.area * 0.001)
    expected = report['config']['school_zone_speed_cap_mps' if school else 'unverified_speed_cap_mps']
    assert cell.attributes['school_zone'] == ('yes' if school else 'no'), cell.id
    assert cell.attributes['speed_limit'] == str(expected) + ' m/s', cell.id
    counts['school_cells' if school else 'non_school_cells'] += 1
    children[int(cell.attributes['parent_lanelet_id'])].append((expected, school))
traffic = rules.create(rules.Locations.Germany, rules.Participants.Vehicle)
for lane in model.laneletLayer:
    expected = lane.attributes['xodr_road_id'] in junctions
    assert lane.attributes['intersection'] == ('yes' if expected else 'no'), lane.id
    counts['intersection_lanelets' if expected else 'non_intersection_lanelets'] += 1
    speeds, zones = zip(*children[lane.id])
    # Lanelet2 Python exposes velocity in km/h; map attributes use m/s.
    assert abs(traffic.speedLimit(lane).speedLimit / 3.6 - min(speeds)) < 1e-6, lane.id
    assert lane.attributes['school_zone'] == ('yes' if all(zones) else 'partial' if any(zones) else 'no')
    mixed += any(zones) and not all(zones)
assert counts['school_cells'] and counts['non_school_cells']
assert counts['school_cells'] + counts['non_school_cells'] == report['cell_count']
if a.baseline:
    before = io.load(str(a.baseline/'hdmap.bin'), io.Origin(0, 0))
    old_report = json.loads((a.baseline/'build_report.json').read_text())
    for key in ('lanelets', 'routing_links', 'stoplines', 'signal_regulations'):
        assert old_report[key] == report[key], key
    for layer_name in ('pointLayer', 'lineStringLayer', 'polygonLayer', 'laneletLayer', 'regulatoryElementLayer'):
        old_layer, new_layer = getattr(before, layer_name), getattr(model, layer_name)
        assert {v.id for v in old_layer} == {v.id for v in new_layer}, layer_name
        allowed = {'speed_limit', 'school_zone', 'speed_source'} if layer_name in ('polygonLayer', 'laneletLayer') else set()
        if layer_name == 'laneletLayer':
            allowed.add('intersection')
        for old in old_layer:
            new = new_layer[old.id]
            assert {k: v for k, v in dict(old.attributes).items() if k not in allowed} == {
                k: v for k, v in dict(new.attributes).items() if k not in allowed}, (layer_name, old.id)
            if layer_name == 'pointLayer':
                assert (old.x, old.y, old.z) == (new.x, new.y, new.z), old.id
            elif layer_name in ('lineStringLayer', 'polygonLayer'):
                assert [v.id for v in old] == [v.id for v in new], old.id
            elif layer_name == 'laneletLayer':
                assert (old.leftBound.id, old.rightBound.id) == (new.leftBound.id, new.rightBound.id)
print(json.dumps({'binary_sha256': sha, **counts, 'mixed_school_lanelets': mixed,
    'non_school_speed_mps': report['config']['unverified_speed_cap_mps'],
    'school_speed_mps': report['config']['school_zone_speed_cap_mps'],
    'geometry_ids_topology_signals_unchanged': bool(a.baseline), 'errors': 0}, indent=2))
