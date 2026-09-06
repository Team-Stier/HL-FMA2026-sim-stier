"""Offline RDB bounding-envelope audit; no simulator connection."""
import argparse
import collections
import gzip
import hashlib
import itertools
import json
import math
import multiprocessing
import os
from pathlib import Path
import time

import lanelet2.io as io
import numpy as np
from shapely import set_precision
from shapely.geometry import MultiPoint, Point, Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree

DIMENSIONS = (3.5190000534057617, 1.6519999504089355, 1.5429999828338623)
OFFSET = (1.284500002861023, 0., 0.)
THRESHOLDS = (.001, .01, .05, .1, .5, 1., 3.)
GRID = .00001
LOCAL = np.array(list(itertools.product([-1, 1], repeat=3))) * np.array(DIMENSIONS) / 2 + OFFSET


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def initialize(map_path):
    global CELLS, POLYS, PLANES, TREE, SOURCE, MAX_PLANE_RESIDUAL
    model = io.load(str(map_path), io.Origin(0, 0))
    SOURCE = {lane.id: {key: lane.attributes[key] for key in
        ('xodr_road_id', 'xodr_section_index', 'xodr_lane_id', 'xodr_s_begin', 'xodr_s_end')}
        for lane in model.laneletLayer}
    CELLS = [c for c in model.polygonLayer if 'type' in c.attributes and c.attributes['type'] == 'hdmap_cell']
    POLYS, PLANES, errors = [], [], []
    for cell in CELLS:
        xyz = np.array([(p.x, p.y, p.z) for p in cell])
        POLYS.append(Polygon(xyz[:, :2]))
        xy, z = xyz[:, :2].mean(axis=0), xyz[:, 2].mean()
        slope = np.linalg.lstsq(xyz[:, :2] - xy, xyz[:, 2] - z, rcond=None)[0]
        PLANES.append((xy, z))
        errors.append(float(np.max(abs((xyz[:, :2] - xy) @ slope + z - xyz[:, 2]))))
    MAX_PLANE_RESIDUAL = max(errors)
    TREE = STRtree(POLYS)


def shapes(state):
    h, p, r = state['hpr']
    ch, sh, cp, sp, cr, sr = math.cos(h), math.sin(h), math.cos(p), math.sin(p), math.cos(r), math.sin(r)
    rotation = np.array([[ch*cp, ch*sp*sr-sh*cr, ch*sp*cr+sh*sr],
                         [sh*cp, sh*sp*sr+ch*cr, sh*sp*cr-ch*sr],
                         [-sp, cp*sr, cp*cr]])
    xyz = np.array(state['xyz'])
    body = MultiPoint((LOCAL @ rotation.T + xyz)[:, :2]).convex_hull
    yaw_local = np.array([[-.475, -.826], [3.044, -.826], [3.044, .826], [-.475, .826]])
    yaw = Polygon(yaw_local @ np.array([[ch, sh], [-sh, ch]]) + xyz[:2])
    return body, yaw, rotation[:, 2]


def evaluate(state):
    body, yaw, normal = shapes(state)
    xyz = np.array(state['xyz'])
    if abs(normal[2]) < .1:
        raise ValueError('Reference-plane height gate invalid for near-vertical vehicle')
    selected, height_distances = [], []
    for index in TREE.query(body.union(yaw)):
        index = int(index)
        xy, z = PLANES[index]
        expected_z = xyz[2] - float((xy - xyz[:2]) @ normal[:2]) / normal[2]
        gap = abs(z - expected_z)
        height_distances.append((index, gap))
        if gap <= .5:
            selected.append(index)
    union = set_precision(unary_union([POLYS[i] for i in selected]), GRID)
    body, yaw = set_precision(body, GRID), set_precision(yaw, GRID)
    outside, yaw_outside = body.difference(union), yaw.difference(union)
    point = Point(*xyz[:2])
    point_distance = point.distance(union) if not union.is_empty else None
    row = {key: state.get(key) for key in
           ('run', 'name', 'target', 'frame', 'sim_time', 'xyz', 'hpr', 'road', 'dimensions', 'center_offset', 'moving', 'step_m')}
    row.update(point_inside_exact=bool(union.covers(point)), point_distance_m=point_distance,
               point_pass_2cm=point_distance is not None and point_distance <= .02,
               bbox_projected_area_m2=body.area, bbox_outside_area_m2=outside.area,
               yaw_outside_area_m2=yaw_outside.area, candidate_cells=len(selected),
               height_rejected_cells=len(height_distances)-len(selected))
    if outside.area > .01 or yaw_outside.area > .01:
        parents = sorted({int(CELLS[i].attributes['parent_lanelet_id']) for i in selected})
        row.update(source_parents={str(parent): SOURCE[parent] for parent in parents},
                   footprint_wkt=body.wkt, outside_wkt=outside.wkt,
                   outside_centroid=[outside.centroid.x, outside.centroid.y] if not outside.is_empty else None,
                   bbox_outside_buffer2cm_area_m2=body.difference(union.buffer(.02)).area)
        for tolerance in (.2, 1.):
            compatible = set_precision(unary_union([POLYS[i] for i, gap in height_distances if gap <= tolerance]), GRID)
            row['outside_height_tolerance_' + str(tolerance) + '_m2'] = body.difference(compatible).area
    return row


def process_batch(batch):
    counts = collections.Counter()
    thresholds = {str(t): collections.Counter() for t in THRESHOLDS}
    events = []
    by_run = {}
    for state in batch:
        row = evaluate(state)
        run = state['run']
        per_run = by_run.setdefault(run, collections.Counter())
        for counter in (counts, per_run):
            counter['retained_drive_samples'] += 1
            counter['moving_samples'] += bool(state['moving'])
            counter['dimensions_observed'] += state.get('dimensions') is not None
            counter['center_offsets_observed'] += state.get('center_offset') is not None
            counter['point_pass_2cm'] += row['point_pass_2cm']
            counter['moving_point_pass_2cm'] += state['moving'] and row['point_pass_2cm']
            counter['point_inside_exact'] += row['point_inside_exact']
        for t in THRESHOLDS:
            c = thresholds[str(t)]
            outside = row['bbox_outside_area_m2'] > t
            yaw_out = row['yaw_outside_area_m2'] > t
            c['all'] += outside
            c['point_pass'] += outside and row['point_pass_2cm']
            c['point_pass_yaw'] += yaw_out and row['point_pass_2cm']
            c['moving'] += outside and state['moving']
            c['moving_point_pass'] += outside and state['moving'] and row['point_pass_2cm']
            c['moving_point_pass_yaw'] += yaw_out and state['moving'] and row['point_pass_2cm']
        if row['bbox_outside_area_m2'] > .01 or row['yaw_outside_area_m2'] > .01:
            events.append(row)
    return dict(counts), {key: dict(value) for key, value in thresholds.items()}, events, {key: dict(value) for key, value in by_run.items()}


def batches(directories, limit=None):
    batch, retained = [], 0
    for directory in directories:
        previous = {}
        with gzip.open(directory / 'rdb_samples.jsonl.gz', 'rt') as stream:
            for line in stream:
                state = json.loads(line)
                if state['phase'] != 'drive':
                    previous.pop(state['name'], None)
                    continue
                if state.get('coord_type') != 0 or state.get('parent') != 0:
                    raise ValueError('Unsupported non-inertial or parent-relative RDB pose')
                if not all(math.isfinite(value) for value in state['xyz'] + state['hpr']):
                    raise ValueError('Non-finite pose')
                dimensions = state.get('dimensions')
                if dimensions is not None and (len(dimensions) != 3 or not all(math.isfinite(v) for v in dimensions) or max(abs(a-b) for a, b in zip(dimensions, DIMENSIONS)) > 1e-6):
                    raise ValueError('Observed vehicle dimensions disagree with verified model')
                center_offset = state.get('center_offset')
                if center_offset is not None and (len(center_offset) != 3 or not all(math.isfinite(v) for v in center_offset) or max(abs(a-b) for a,b in zip(center_offset, OFFSET)) > 1e-6):
                    raise ValueError('Observed center_offset disagrees with verified model')
                old = previous.get(state['name'])
                moving, step = False, 0.
                if old and old['target'] == state['target']:
                    dt = state['sim_time'] - old['sim_time']
                    step = math.dist(state['xyz'][:2], old['xyz'][:2])
                    moving = 0 < dt <= .5 and .001 < step <= max(.5, 20 * dt)
                previous[state['name']] = state
                state.update(run=str(directory), moving=moving, step_m=step)
                batch.append(state)
                retained += 1
                if len(batch) == 2000:
                    yield batch
                    batch = []
                if limit and retained >= limit:
                    if batch:
                        yield batch
                    return
    if batch:
        yield batch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--replay-summary', type=Path, required=True)
    parser.add_argument('--extra-run', type=Path, action='append', default=[])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=4, choices=range(1, 5))
    parser.add_argument('--limit', type=int)
    args = parser.parse_args()
    started = time.perf_counter()
    model = Path('/home/stier/VIRES/VTD.2025.2/Data/Distros/Current/Config/Players/Vehicles/SmartForTwo_14.xml')
    evidence = Path('/home/stier/HL-FMA2026-sim-stier/docs/object-offset-measurement.json')
    expected_model = '7ae36b6495f7806f0e23cc848894571ca7470f01a47bb52f767218756025b7fb'
    assert digest(model) == expected_model, 'Vehicle model changed since independent offset measurement'
    replay = json.loads(args.replay_summary.read_text())
    map_sha256 = digest(args.map)
    if replay.get('map_sha256') != map_sha256:
        raise ValueError('Replay manifest map_sha256 does not match the requested map')
    directories, empty_runs_without_logs = [], []
    archived = Path(__file__).resolve().parent
    for run in replay['runs'] + [{'directory': str(p), 'counts': None} for p in args.extra_run]:
        directory = Path(run['directory'])
        if not (directory / 'rdb_samples.jsonl.gz').exists():
            fallback = archived / directory.name
            if (fallback / 'rdb_samples.jsonl.gz').exists():
                directory = fallback
            elif run.get('counts') is not None and run['counts'].get('retained_drive_samples', 0) == 0:
                empty_runs_without_logs.append(str(directory))
                continue
            else:
                raise FileNotFoundError(f'Missing nonempty/requested run log: {directory}')
        if directory not in directories:
            directories.append(directory)
    args.output.mkdir(parents=True, exist_ok=True)
    os.nice(5)
    initialize(args.map)
    # One analytical check: reference origin and forward offset, including full HPR corner projection.
    body, yaw, _ = shapes({'xyz': [0., 0., 0.], 'hpr': [0., 0., 0.]})
    assert abs(body.area - DIMENSIONS[0]*DIMENSIONS[1]) < 1e-8
    assert max(abs(a-b) for a,b in zip(body.bounds, (-.475, -.826, 3.044, .826))) < 1e-6
    assert body.symmetric_difference(yaw).area < 1e-6
    counts = collections.Counter()
    thresholds = {str(t): collections.Counter() for t in THRESHOLDS}
    per_run, groups, worst, event_count = {}, {}, [], 0
    events_path = args.output / 'violations.jsonl.gz'
    with gzip.open(events_path, 'wt') as events_file:
        with multiprocessing.get_context('fork').Pool(args.workers) as executor:
            for batch_counts, batch_thresholds, events, run_counts in executor.imap(process_batch, batches(directories, args.limit)):
                counts.update(batch_counts)
                for key, value in batch_thresholds.items():
                    thresholds[key].update(value)
                for key, value in run_counts.items():
                    per_run.setdefault(key, collections.Counter()).update(value)
                for row in events:
                    events_file.write(json.dumps(row, separators=(',', ':')) + '\n')
                    event_count += 1
                    if row['moving'] and row['point_pass_2cm'] and row['bbox_outside_area_m2'] > .05:
                        key = (row['run'], row['target'], tuple(row['road'][:2]) if row['road'] else None)
                        group = groups.setdefault(key, {'run': row['run'], 'target': row['target'],
                            'reported_road_lane': row['road'][:2] if row['road'] else None, 'samples': 0, 'worst': row})
                        group['samples'] += 1
                        if row['bbox_outside_area_m2'] > group['worst']['bbox_outside_area_m2']:
                            group['worst'] = row
                        worst.append(row)
                worst = sorted(worst, key=lambda row: row['bbox_outside_area_m2'], reverse=True)[:50]
                if counts['retained_drive_samples'] % 20000 == 0:
                    print(json.dumps({'processed': counts['retained_drive_samples'], 'seconds': time.perf_counter()-started}), flush=True)
    result = {'map': str(args.map.resolve()), 'map_sha256': digest(args.map), 'replay_summary': str(args.replay_summary),
        'model': str(model), 'model_sha256': digest(model), 'offset_evidence': str(evidence), 'offset_evidence_sha256': digest(evidence),
        'script': str(Path(__file__).resolve()), 'script_sha256': digest(__file__),
        'empty_runs_without_logs': empty_runs_without_logs,
        'runs': [{'directory': str(path), 'log_sha256': digest(path/'rdb_samples.jsonl.gz'), 'counts': dict(per_run.get(str(path), {}))} for path in directories],
        'bbox_dimensions_m': DIMENSIONS, 'bbox_center_offset_local_m': OFFSET, 'grid_precision_m': GRID,
        'height_tolerance_m': .5, 'cell_plane_max_residual_m': MAX_PLANE_RESIDUAL, 'workers': args.workers,
        'elapsed_seconds': time.perf_counter()-started, 'counts': dict(counts),
        'outside_thresholds_m2': {key: dict(value) for key, value in thresholds.items()},
        'violation_rows_over_0_01_m2': event_count, 'violation_log': str(events_path.resolve()), 'violation_log_sha256': digest(events_path),
        'point_pass_moving_groups_over_0_05_m2': sorted(groups.values(), key=lambda g: g['worst']['bbox_outside_area_m2'], reverse=True),
        'worst_point_pass_moving_samples': worst,
        'method': 'All retained phase=drive poses are tested; moving subset exactly uses replay consecutive-sample dt<=.5s, step>.001m and step<=max(.5,20dt). Dimensions verified when logged; unchanged model hash and prior RDB observation establish missing dimensions/offset. Rz(h)Ry(p)Rx(r) projects all8 bbox corners to convex XY envelope; yaw-only rectangle measured independently. Driving cell union uses cell centroid surface Z within.5m of vehicle reference local-z=0 plane, with.2/1m sensitivity for events. GEOS overlay snaps to10micrometre precision. Non-driving map areas are not added to accommodate vehicle departures. Model bbox is not measured visual mesh or wheel contact; source type corroboration requires separate review.'}
    (args.output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'summary': str(args.output/'summary.json'), 'counts': dict(counts), 'thresholds': result['outside_thresholds_m2'], 'seconds': result['elapsed_seconds']}), flush=True)


if __name__ == '__main__':
    main()
