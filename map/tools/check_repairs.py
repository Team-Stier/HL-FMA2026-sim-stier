"""Verify the reviewed stopline and non-driving corrections in a generated map."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import lanelet2.io as io
import lanelet2.routing as routing
import lanelet2.traffic_rules as traffic_rules


def check(directory, mesh_path):
    report = json.loads((directory / 'build_report.json').read_text())
    binary_sha256 = hashlib.sha256((directory / 'hdmap.bin').read_bytes()).hexdigest()
    assert binary_sha256 == report['binary_sha256'], 'Report does not match the binary being checked'
    markings = json.loads(mesh_path.read_text())
    model = io.load(str(directory / 'hdmap.bin'), io.Origin(0., 0.))
    stops = {int(line.attributes['xodr_object_id']): line for line in model.lineStringLayer
        if 'type' in line.attributes and line.attributes['type'] == 'stop_line' and 'xodr_object_id' in line.attributes}
    corrections = []
    for override in report['config']['object_lateral_overrides']:
        index = {5153: 125, 5238: 317, 5161: 128, 1386: 214, 2935: 517}[override['object_id']]
        stop = stops[override['object_id']]
        center = [sum(getattr(point, axis) for point in stop) / len(stop) for axis in ('x', 'y', 'z')]
        distance = math.dist(center, markings[index]['center'])
        assert distance < .015, (override, distance)
        assert math.isclose(float(stop.attributes['xodr_t']), override['source_t'], abs_tol=1e-7)
        assert float(stop.attributes['effective_t']) == override['t']
        assert stop.attributes['cell_assignment'] == ('projected_to_approach_end'
            if 'mesh_candidate_index' in override else 'projected_to_lane')
        corrections.append({'object': override['object_id'], 'mesh': index, 'center_error_m': distance})
    restored = {25,26,581,187,186,69,198,280,207,409,520,451,334,576,599,608,609,
        71,315,36,261,30,41,632,666,125,440,237,455,144}
    generated = {row['mesh_candidate_index'] for row in report['stoplines'] if 'mesh_candidate_index' in row}
    assert restored <= generated, sorted(restored - generated)
    assert not generated.intersection(range(106,116)), 'Crosswalk stripes must not be stoplines'
    for override in report['config']['non_driving_lane_overrides']:
        key = [override['road_id'], override['section_index'], override['lane_id']]
        assert not any(row['xodr'] == key for row in report['lanelets'].values()), key
    contacts = report['rejected_native_point_contacts']
    assert len(contacts) == 1, contacts
    graph = routing.RoutingGraph(model, traffic_rules.create(
        traffic_rules.Locations.Germany, traffic_rules.Participants.Vehicle))
    for contact in contacts:
        source, target = model.laneletLayer[contact['from']], model.laneletLayer[contact['to']]
        assert target.id not in {lane.id for lane in graph.following(source, False)}, contact
        assert [source.attributes['xodr_road_id'], target.attributes['xodr_road_id']] == ['1196', '1218']
        assert math.dist([source.leftBound[-1].x, source.leftBound[-1].y],
                         [target.leftBound[0].x, target.leftBound[0].y]) < 1e-8
    result = {'binary_sha256': binary_sha256, 'lateral_corrections': corrections,
        'restored_stoplines_verified': len(restored), 'crosswalk_false_positives': 0,
        'implicit_zero_width_connections_rejected': len(contacts),
        'non_driving_overrides_verified': len(report['config']['non_driving_lane_overrides'])}
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, nargs='?', default=Path('map'))
    parser.add_argument('--mesh', type=Path, default=Path('map/stopline_mesh.json'))
    args = parser.parse_args()
    check(args.directory, args.mesh)
