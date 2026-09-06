"""Drive temporary native VTD NPCs; save RDB evidence, separate placement from motion.

Source the ROS workspace and add install/hdmap_core/lib/python3.12/site-packages
before running. This controls only MapAudit* NPCs and stops VTD when finished.
"""
import argparse
import bisect
import collections
import gzip
import hashlib
import json
import math
from pathlib import Path
import random
import select
import socket
import struct
import sys
import time
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'config/tools'))
from measure_object_offsets import SCP, RDB, ENTRY, STATE

ROAD = struct.Struct('<IHbB6fBBHf')


def packets(buffer, roads=None):
    roads = {} if roads is None else roads
    while len(buffer) >= RDB.size:
        magic, _, header_size, data_size, frame, sim_time = RDB.unpack_from(buffer)
        total = header_size + data_size
        if magic != 35712 or header_size < RDB.size or total > 64 * 1024 * 1024:
            raise ValueError('Invalid RDB header')
        if len(buffer) < total:
            return
        packet = bytes(buffer[:total])
        del buffer[:total]
        states = []
        cursor = header_size
        while cursor < total:
            if total - cursor < ENTRY.size:
                raise ValueError('Truncated RDB entry')
            eh, ed, size, kind, _ = ENTRY.unpack_from(packet, cursor)
            if eh < ENTRY.size or cursor + eh + ed > total or (ed and (not size or ed % size)):
                raise ValueError('Invalid RDB entry size')
            if kind in (5, 9) and size >= (ROAD.size if kind == 5 else STATE.size):
                for offset in range(cursor + eh, cursor + eh + ed, size):
                    if kind == 5:
                        road = ROAD.unpack_from(packet, offset)
                        roads[road[0]] = [road[1], road[2], road[4], road[5], road[6]]
                    else:
                        state = STATE.unpack_from(packet, offset)
                        name = state[4].split(b'\0')[0].decode(errors='replace')
                        if name.startswith('MapAudit'):
                            if not all(math.isfinite(v) for v in state[11:17]):
                                raise ValueError('Non-finite RDB position')
                            states.append({'name': name, 'id': state[0], 'xyz': state[11:14],
                                           'hpr': state[14:17], 'dimensions': state[5:8], 'center_offset': state[8:11], 'coord_type': state[18],
                                           'parent': state[20]})
            cursor += eh + ed
        for state in states:
            state.update(frame=frame, sim_time=sim_time, road=roads.get(state['id']))
        yield states


def capture(connection, buffer, seconds, callback):
    roads = {}
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if not select.select([connection], [], [], min(0.2, max(0., end - time.monotonic())))[0]:
            continue
        data = connection.recv(4 * 1024 * 1024)
        if not data:
            raise ConnectionError('RDB stream closed')
        buffer.extend(data)
        for states in packets(buffer, roads):
            for state in states:
                callback(state)


def self_check():
    road = ROAD.pack(8, 465, -1, 0, 20., -1.5, 0., 0., 0., 0., 0, 0, 0, 0.)
    state = STATE.pack(8, 1, 1, 0, b'MapAuditCheck', 2.7, 1.5, 1.5, 0., 0., .75,
                       10., 20., 30., .1, 0., 0., 0, 0, 0, 0, 0, 1)
    data = ENTRY.pack(ENTRY.size, len(road), len(road), 5, 0) + road
    data += ENTRY.pack(ENTRY.size, len(state), len(state), 9, 0) + state
    raw = RDB.pack(35712, 1, RDB.size, len(data), 7, 1.2) + data
    buffer = bytearray(raw[:20])
    assert list(packets(buffer)) == []
    buffer.extend(raw[20:] + raw)
    decoded = list(packets(buffer))
    assert not buffer and len(decoded) == 2
    assert decoded[0][0]['xyz'] == (10., 20., 30.)
    assert decoded[0][0]['center_offset'] == (0., 0., .75)
    assert decoded[0][0]['road'] == [465, -1, 20., -1.5, 0.]
    roads = {}
    road_packet = ENTRY.pack(ENTRY.size, len(road), len(road), 5, 0) + road
    raw_road = RDB.pack(35712, 1, RDB.size, len(road_packet), 7, 1.2) + road_packet
    assert list(packets(bytearray(raw_road), roads)) == [[]]
    assert roads[8][:3] == [465, -1, 20.]
    try:
        list(packets(bytearray(b'\0' * RDB.size)))
    except ValueError:
        pass
    else:
        raise AssertionError('Invalid header accepted')
    source = ET.fromstring('<road><lanes><laneOffset s="0" a="1" b="0.1" c="0" d="0"/><laneSection s="0"><right><lane id="-1" type="driving"><width sOffset="0" a="3" b="0" c="0" d="0"/></lane><lane id="-2" type="driving"><width sOffset="0" a="2" b="0.2" c="0" d="0"/></lane></right></laneSection></lanes></road>')
    assert abs(lane_center_t({1: source}, 1, -1, 10.)[0] - .5) < 1e-12
    assert abs(lane_center_t({1: source}, 1, -2, 10.)[0] + 3.) < 1e-12
    print('RDB packets, road association, malformed input and source lane center t: OK')


def lane_center_t(roads, road_id, lane_id, s):
    """Explicit source placement; VTD may reselect overlapping roads. Returns t and width."""
    def polynomial(items, coordinate, start):
        item = next((item for item in reversed(items) if float(item.get(start)) <= coordinate + 1e-7), None)
        if item is None:
            return 0.
        ds = coordinate - float(item.get(start))
        return sum(float(item.get(key, '0')) * ds ** power for power, key in enumerate('abcd'))

    road = roads[road_id]
    sections = road.findall('lanes/laneSection')
    section = next(item for item in reversed(sections) if float(item.get('s')) <= s + 1e-7)
    offset = polynomial(road.findall('lanes/laneOffset'), s, 's')
    ds = s - float(section.get('s'))
    sign = 1 if lane_id > 0 else -1
    lanes = sorted(section.findall(('left' if lane_id > 0 else 'right') + '/lane'),
                   key=lambda item: abs(int(item.get('id'))))
    for lane in lanes:
        if not lane.findall('width'):
            raise ValueError(f'Lane width missing: road {road_id} lane {lane.get("id")}')
        width = max(0., polynomial(lane.findall('width'), ds, 'sOffset'))
        if int(lane.get('id')) == lane_id:
            if lane.get('type') != 'driving':
                raise ValueError(f'Target is not driving: road {road_id} lane {lane_id}')
            return offset + sign * width * .5, width
        offset += sign * width
    raise ValueError(f'No source lane {lane_id} in road {road_id} at s={s}')


def audit(args):
    import hdmap
    from lanelet2.core import BasicPoint2d
    from lanelet2.geometry import findWithin2d

    model = hdmap.hdmap_init(str(args.map))
    native, cells = model.laneletMap(), model.cells()
    source_roads = {int(road.get('id')): road for road in ET.parse(args.xodr).getroot().findall('road')}
    targets = []
    for lane in native.laneletLayer:
        attrs = lane.attributes
        road, section, lane_id = (int(attrs[key]) for key in ('xodr_road_id', 'xodr_section_index', 'xodr_lane_id'))
        begin, end = float(attrs['xodr_s_begin']), float(attrs['xodr_s_end'])
        margin = min(1., (end - begin) * 0.1)
        targets.append({'target_lanelet': lane.id, 'source_road': road, 'source_section': section,
                        'source_lane': lane_id, 's_range': [begin, end],
                        'placement_s': (begin + (end - begin) * args.placement_fraction if args.placement_fraction is not None
                                        else end - margin if lane_id > 0 else begin + margin)})
    if args.target_plan:
        plan = json.loads(args.target_plan.read_text())
        if plan['map_sha256'] != hashlib.sha256(args.map.read_bytes()).hexdigest():
            raise ValueError('Target plan map SHA differs from selected map')
        templates = {target['target_lanelet']: target for target in targets}
        selected = []
        for index, item in enumerate(plan['targets']):
            target = dict(templates[item['target_lanelet']], placement_s=float(item['placement_s']), plan_index=index)
            if not math.isfinite(target['placement_s']) or not target['s_range'][0] <= target['placement_s'] <= target['s_range'][1]:
                raise ValueError(f'Placement s outside target lanelet: {item}')
            selected.append(target)
        targets = selected
    for target in targets:
        target['placement_t'], target['source_lane_width_m'] = lane_center_t(source_roads, target['source_road'], target['source_lane'], target['placement_s'])
    targets.sort(key=lambda target: target['target_lanelet'])
    random.Random(20260905).shuffle(targets)
    all_road_ids = sorted({target['source_road'] for target in targets})
    if args.lanelet_ids:
        requested = set(json.loads(args.lanelet_ids.read_text()))
        missing = requested - {target['target_lanelet'] for target in targets}
        if missing:
            raise ValueError(f'Unknown lanelet IDs: {sorted(missing)}')
        targets = [target for target in targets if target['target_lanelet'] in requested]
    targets = targets[args.offset:args.offset + args.limit] if args.limit else targets[args.offset:]
    if not targets:
        raise ValueError('No targets selected')
    args.output.mkdir(parents=True, exist_ok=True)
    summary = {'map': str(args.map.resolve()), 'map_sha256': hashlib.sha256(args.map.read_bytes()).hexdigest(),
               'xodr': str(args.xodr.resolve()), 'xodr_sha256': hashlib.sha256(args.xodr.read_bytes()).hexdigest(),
               'placement_mode': 'explicit_source_lane_center_t',
               'target_plan': str(args.target_plan) if args.target_plan else None,
               'placement_source_s_tolerance_m': 1., 'placement_source_t_tolerance_m': .2,
               'source_width_policy': 'original XML, no HD map width_start_overrides',
               'preplacement_min_source_width_m': 1.852 if args.require_placement else None,
               'evidence_all_samples': args.all_samples, 'require_verified_placement': args.require_placement,
               'source_lanelets': len(native.laneletLayer), 'source_roads': all_road_ids,
               'source_cells': len(cells), 'targets': len(targets), 'fleet_size': args.fleet,
               'seconds_per_stint_wall': args.seconds, 'fresh_npc_per_batch': args.fresh,
               'placement_fraction': args.placement_fraction, 'desired_speed_mps': args.speed,
               'placement_excluded_from_distance': True, 'driver_obeys_signs_and_lights': True,
               'cell_probe_half_width_m': .02, 'cell_probe_z_tolerance_m': .5,
               'lane_polygon_xy_tolerance_m': .02, 'cleanup_sent': False}
    names = [f'MapAudit{index:03d}' for index in range(min(args.fleet, len(targets)))]
    results, buffer = [], bytearray()
    driven_lanes, driven_cells, driven_roads, rdb_roads = set(), set(), set(), set()
    road_by_lane = {lane.id: int(lane.attributes['xodr_road_id']) for lane in native.laneletLayer}
    lane_by_lanelet = {lane.id: int(lane.attributes['xodr_lane_id']) for lane in native.laneletLayer}
    counters = collections.Counter()
    active, previous, last_log = {}, {}, {}
    phase = 'setup'
    started = time.monotonic()
    connection = socket.create_connection(('127.0.0.1', 48190), timeout=3.)
    connection.setblocking(False)
    evidence = gzip.open(args.output / 'rdb_samples.jsonl.gz', 'wt')
    control = socket.create_connection(('127.0.0.1', 48179), timeout=3.)

    def command(xml):
        data = xml.encode()
        control.sendall(SCP.pack(40108, 1, b'MapAudit', b'TaskControl', len(data)) + data)

    def receive(state):
        name = state['name']
        if name not in active:
            return
        result = active[name]
        if result.get('placement_preblocked') or phase == 'drive' and result.get('placement_blocked'):
            return
        if state['coord_type'] != 0 or state['parent'] != 0:
            raise ValueError('Expected inertial RDB reference coordinates')
        x, y, z = state['xyz']
        point = BasicPoint2d(x, y)
        lanes = {lane.id for _, lane in findWithin2d(native.laneletLayer, point, .02)}
        footprint = [BasicPoint2d(x + dx, y + dy) for dx, dy in [(-.02, -.02), (.02, -.02), (.02, .02), (-.02, .02)]]
        cell_ids = set(model.cellTree().queryOverlaps(footprint, z - .5, z + .5))
        cell_lanes = {cells[index].parent().lanelet_id for index in cell_ids}
        road = state['road']
        matching_lanes = {lane for lane in cell_lanes if road and
                          (road_by_lane[lane], lane_by_lanelet[lane]) == tuple(road[:2])}
        matching_cells = {index for index in cell_ids if cells[index].parent().lanelet_id in matching_lanes}
        counters['samples'] += 1
        if phase == 'placement':
            result['placement'] = dict(state, lanelet_hits=sorted(lanes), cell_hits=sorted(cell_ids))
        elif phase == 'drive':
            result['samples'] += 1
            result['lane_misses'] += not lanes
            result['cell_misses'] += not cell_ids
            result['source_lane_misses'] += bool(road) and not matching_lanes
            result['target_hits'] += result['target_lanelet'] in cell_lanes
            result.setdefault('first_drive', state)
            result['last_drive'] = state
            previous_state = previous.get(name)
            if previous_state:
                old, old_lanes, old_cells = previous_state
                dt = state['sim_time'] - old['sim_time']
                distance = math.dist(state['xyz'][:2], old['xyz'][:2])
                if 0 < dt <= .5 and distance <= max(.5, 20 * dt):
                    result['distance_m'] += distance
                    if distance > .001:
                        driven_lanes.update(matching_lanes | old_lanes)
                        driven_cells.update(matching_cells | old_cells)
                        driven_roads.update(road_by_lane[lane] for lane in matching_lanes | old_lanes)
                        if state['road']:
                            rdb_roads.add(state['road'][0])
                        if old['road']:
                            rdb_roads.add(old['road'][0])
                        if result['target_lanelet'] in matching_lanes & old_lanes:
                            result['target_distance_m'] += distance
                else:
                    result['excluded_jumps'] += 1
            previous[name] = (state, matching_lanes, matching_cells)
        anomaly = phase == 'drive' and (not lanes or not cell_ids)
        if args.all_samples or anomaly or state['sim_time'] - last_log.get(name, -1e9) >= .099:
            evidence.write(json.dumps(dict(state, phase=phase, target=result['target_lanelet'],
                                           lanelets=sorted(lanes), cells=sorted(cell_ids))) + '\n')
            last_log[name] = state['sim_time']

    try:
        def create_players():
            for name in names:
                command(f'<Player name="{name}"><Create category="vehicle" vehicle="SmartForTwo_14_WhiteBlack" control="internal"/></Player>')
            capture(connection, buffer, 3., receive)

        create_players()
        for offset in range(0, len(targets), len(names)):
            if args.fresh and offset:
                for name in names:
                    command(f'<Player name="{name}"><Delete/></Player>')
                capture(connection, buffer, .3, receive)
                create_players()
            batch = targets[offset:offset + len(names)]
            active.clear(); previous.clear(); last_log.clear()
            phase = 'setup'
            for name in names:
                command(f'<Player name="{name}"><DriverBehavior desiredSpeed="0" obeyTrafficSigns="true" obeyTrafficLights="true"/></Player>')
                command(f'<Set entity="player" name="{name}"><Speed value="0"/></Set>')
            capture(connection, buffer, .2, receive)
            for name, target in zip(names, batch):
                active[name] = dict(target, name=name, samples=0, distance_m=0., target_distance_m=0.,
                                    lane_misses=0, cell_misses=0, source_lane_misses=0, target_hits=0, excluded_jumps=0)
                if args.require_placement and target['source_lane_width_m'] < 1.852:
                    # Fixed NPC model width is 1.652m in its measured RDB geometry.
                    active[name].update(placement_preblocked=True, placement_blocked=True, placement_verified=False)
                    command(f'<Player name="{name}"><Delete/></Player>')
                    continue
                command(f'<Set entity="player" name="{name}"><TrackPos track="{target["source_road"]}" lane="{target["source_lane"]}" t="{target["placement_t"]}" s="{target["placement_s"]}" dhDeg="{180 if target["source_lane"] > 0 else 0}"/></Set>')
            phase = 'placement'
            capture(connection, buffer, .7, receive)
            for result in active.values():
                if result.get('placement_preblocked'):
                    continue
                placement = result.get('placement')
                road = placement.get('road') if placement else None
                expected_t = None
                if road:
                    try:
                        expected_t = lane_center_t(source_roads, road[0], road[1], road[2])[0]
                    except (KeyError, ValueError, StopIteration):
                        pass
                result['placement_s_delta_m'] = road[2] - result['placement_s'] if road else None
                result['source_center_t_at_received_s'] = expected_t
                result['placement_verified'] = bool(placement and road and expected_t is not None and
                    result['source_lane_width_m'] > 1e-6 and abs(result['placement_s_delta_m']) <= 1. and
                    road[:2] == [result['source_road'], result['source_lane']] and
                    result['target_lanelet'] in placement['lanelet_hits'] and
                    abs(road[3] - expected_t) <= .2)
                result['placement_blocked'] = bool(args.require_placement and (not result['placement_verified'] or
                    result['source_lane_width_m'] < placement['dimensions'][1] + .2))
                if result['placement_blocked']:
                    command(f'<Player name="{result["name"]}"><Delete/></Player>')
            if not any(result.get('placement') for result in active.values()) and not all(
                    result.get('placement_preblocked') for result in active.values()):
                raise ConnectionError('No placement observations; simulator stream may have stopped')
            phase = 'drive'
            for name in active:
                if active[name]['placement_blocked']:
                    continue
                command(f'<Player name="{name}"><DriverBehavior desiredSpeed="{args.speed}" obeyTrafficSigns="true" obeyTrafficLights="true"/></Player>')
            capture(connection, buffer, args.seconds, receive)
            for result in active.values():
                result['sim_duration_s'] = (result['last_drive']['sim_time'] - result['first_drive']['sim_time']
                                            if result['samples'] else 0.)
                result['requested_duration_completed'] = result['sim_duration_s'] >= args.seconds * .9
                result['status'] = ('placement_mismatch' if result.get('placement_blocked') else 'no_rdb' if not result['samples'] else 'stream_stalled' if not result['requested_duration_completed']
                                    else 'no_motion' if result['distance_m'] < .05
                                    else 'map_miss' if result['cell_misses'] or result['lane_misses'] else
                                    'target_not_driven' if result['target_distance_m'] < min(.05, .2 * (result['s_range'][1] - result['s_range'][0])) else 'sampled_motion')
                results.append(result)
            active.clear()
            print(json.dumps({'completed': len(results), 'total': len(targets), 'moving_lanelets': len(driven_lanes),
                              'moving_roads': len(driven_roads), 'moving_cells': len(driven_cells),
                              'distance_m': round(sum(item['distance_m'] for item in results), 2),
                              'status': dict(collections.Counter(item['status'] for item in results)),
                              'elapsed_s': round(time.monotonic() - started, 1)}), flush=True)
            (args.output / 'stints.json').write_text(json.dumps(results, indent=2))
            if all(result['status'] in ('no_rdb', 'stream_stalled') for result in results[-len(batch):]):
                raise ConnectionError('No audit NPC RDB states; simulator stream may have stopped')
    finally:
        for name in names:
            try:
                command(f'<Player name="{name}"><Delete/></Player>')
            except OSError as error:
                summary.setdefault('cleanup_errors', []).append(str(error))
        try:
            command('<SimCtrl><Stop/></SimCtrl>')
            capture(connection, buffer, .3, lambda state: None)
        finally:
            connection.close(); control.close(); evidence.close()
            summary.update(completed=len(results), status=dict(collections.Counter(item['status'] for item in results)),
                           sampled_distance_m=sum(item['distance_m'] for item in results),
                           placement_verified_count=sum(item.get('placement_verified', False) for item in results),
                           moving_lanelets=sorted(driven_lanes), moving_cells=sorted(driven_cells),
                           moving_roads=sorted(driven_roads), road_coverage_source="RDB_road_and_lane_confirmed_map_cell_parent",
                           rdb_reported_moving_roads=sorted(rdb_roads), samples=counters['samples'],
                           elapsed_wall_s=time.monotonic() - started, cleanup_sent=True)
            (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
            (args.output / 'stints.json').write_text(json.dumps(results, indent=2))


def cell_headings(model):
    """Travel tangent at each cell midpoint, using the builder's sampled boundaries."""
    by_lane = collections.defaultdict(list)
    for cell in model.cells():
        by_lane[cell.parent().lanelet_id].append(cell)
    headings = {}
    for lane in model.laneletMap().laneletLayer:
        centers = [((a.x + b.x) * .5, (a.y + b.y) * .5) for a, b in zip(lane.leftBound, lane.rightBound)]
        distances = [0.]
        for a, b in zip(centers, centers[1:]):
            distances.append(distances[-1] + math.dist(a, b))
        station = 0.
        for cell in sorted(by_lane[lane.id], key=lambda value: value.parent().index_in_lanelet):
            length = float(cell.polygon3d().attributes['centerline_length_m'])
            index = min(len(centers) - 2, max(0, bisect.bisect_right(distances, station + length * .5) - 1))
            a, b = centers[index:index + 2]
            headings[cell.id] = math.atan2(b[1] - a[1], b[0] - a[0])
            station += length
    return headings


def replay(args):
    """Re-query retained inertial RDB samples against a map without controlling VTD."""
    import hdmap
    from lanelet2.core import BasicPoint2d
    from lanelet2.geometry import findWithin2d

    model = hdmap.hdmap_init(str(args.map))
    native, cells = model.laneletMap(), model.cells()
    source = {lane.id: (int(lane.attributes['xodr_road_id']), int(lane.attributes['xodr_lane_id']))
              for lane in native.laneletLayer}
    covered_lanes, covered_cells, covered_roads = set(), set(), set()
    forward_lanes, forward_cells, forward_roads = set(), set(), set()
    directions = {index: (math.cos(heading), math.sin(heading)) for index, heading in cell_headings(model).items()}
    counts, runs, misses = collections.Counter(), [], []
    for directory in args.replay:
        previous, run_counts = {}, collections.Counter()
        distance = 0.
        with gzip.open(directory / 'rdb_samples.jsonl.gz', 'rt') as evidence:
            for line in evidence:
                state = json.loads(line)
                if state['phase'] != 'drive':
                    previous.pop(state['name'], None)
                    continue
                x, y, z = state['xyz']
                point = BasicPoint2d(x, y)
                lanes = {lane.id for _, lane in findWithin2d(native.laneletLayer, point, .02)}
                footprint = [BasicPoint2d(x + dx, y + dy) for dx, dy in
                             [(-.02, -.02), (.02, -.02), (.02, .02), (-.02, .02)]]
                hits = set(model.cellTree().queryOverlaps(footprint, z - .5, z + .5))
                road = state['road']
                matching = {index for index in hits if road and
                            source[cells[index].parent().lanelet_id] == tuple(road[:2])}
                run_counts['retained_drive_samples'] += 1
                run_counts['lane_miss_samples'] += not lanes
                run_counts['cell_miss_samples'] += not hits
                run_counts['source_lane_miss_samples'] += bool(road) and not matching
                if not lanes or not hits:
                    misses.append(dict(state, run=str(directory), repaired_lanelets=sorted(lanes),
                                       repaired_cells=sorted(hits)))
                old = previous.get(state['name'])
                if old and old[0]['target'] == state['target']:
                    dt = state['sim_time'] - old[0]['sim_time']
                    step = math.dist(state['xyz'][:2], old[0]['xyz'][:2])
                    if 0 < dt <= .5 and step <= max(.5, 20 * dt):
                        distance += step
                        if step > .001:
                            covered_cells.update(matching | old[1])
                            parents = {cells[index].parent().lanelet_id for index in matching | old[1]}
                            covered_lanes.update(parents)
                            covered_roads.update(source[lane][0] for lane in parents)
                            dx, dy = x - old[0]['xyz'][0], y - old[0]['xyz'][1]
                            def aligned(indices, heading):
                                hx, hy = math.cos(heading), math.sin(heading)
                                return {index for index in indices if
                                        dx * directions[index][0] + dy * directions[index][1] > 0. and
                                        hx * directions[index][0] + hy * directions[index][1] > 0.}
                            current_forward = aligned(matching, state['hpr'][0])
                            old_forward = aligned(old[1], old[0]['hpr'][0])
                            run_counts['source_matched_direction_conflict_samples'] += bool(matching) and not current_forward
                            forward_cells.update(current_forward | old_forward)
                            parents = {cells[index].parent().lanelet_id for index in current_forward | old_forward}
                            forward_lanes.update(parents)
                            forward_roads.update(source[lane][0] for lane in parents)
                    else:
                        run_counts['excluded_jumps'] += 1
                previous[state['name']] = state, matching
        counts.update(run_counts)
        runs.append(dict(directory=str(directory), counts=dict(run_counts),
                         retained_sample_distance_m=distance))
    args.output.mkdir(parents=True, exist_ok=True)
    summary = dict(map=str(args.map.resolve()), map_sha256=hashlib.sha256(args.map.read_bytes()).hexdigest(),
                   source_lanelets=len(source), source_cells=len(cells), source_roads=sorted({v[0] for v in source.values()}),
                   counts=dict(counts), runs=runs, moving_lanelets=sorted(covered_lanes),
                   moving_cells=sorted(covered_cells), moving_roads=sorted(covered_roads),
                   uncovered_roads=sorted({v[0] for v in source.values()} - covered_roads),
                   uncovered_lanelets=sorted(set(source) - covered_lanes),
                   coverage_source='retained RDB inertial motion with exact RDB road/lane matching map cell parent',
                   direction_filtered_moving_cells=sorted(forward_cells),
                   direction_filtered_moving_lanelets=sorted(forward_lanes),
                   direction_filtered_moving_roads=sorted(forward_roads),
                   direction_filter='Both body heading and actual XY motion must have positive dot product with each accepted cell travel tangent',
                   placement_excluded=True, whole_continuous_route_verified=False)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
    (args.output / 'misses.json').write_text(json.dumps(misses, indent=2))
    print(json.dumps(dict(map_sha256=summary['map_sha256'], counts=summary['counts'],
                         moving_roads=len(covered_roads), moving_lanelets=len(covered_lanes),
                         moving_cells=len(covered_cells), direction_filtered_cells=len(forward_cells),
                         direction_filtered_roads=len(forward_roads), uncovered_roads=summary['uncovered_roads'])), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--xodr', type=Path, help='Original OpenDRIVE; required for live explicit t placement')
    parser.add_argument('--fleet', type=int, default=32)
    parser.add_argument('--seconds', type=float, default=6.)
    parser.add_argument('--speed', type=float, default=6.)
    parser.add_argument('--offset', type=int, default=0)
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--target-plan', type=Path, help='JSON map SHA and target_lanelet/placement_s entries')
    parser.add_argument('--lanelet-ids', type=Path, help='JSON array of lanelet IDs to sample')
    parser.add_argument('--placement-fraction', type=float, help='Fraction along source s range; default is travel ingress')
    parser.add_argument('--require-placement', action='store_true', help='Delete NPCs with unverified or too-narrow placement before driving')
    parser.add_argument('--all-samples', action='store_true', help='Retain every received audit RDB state')
    parser.add_argument('--fresh', action='store_true', help='Delete and recreate NPCs before each batch')
    parser.add_argument('--replay', type=Path, nargs='+', help='Saved run directories; offline map query only')
    parser.add_argument('--self-check', action='store_true')
    args = parser.parse_args()
    if args.require_placement:
        args.fresh = True
    if args.self_check:
        self_check()
    else:
        if not args.map or not args.output or not 1 <= args.fleet <= 128 or args.offset < 0 or args.limit < 0:
            parser.error('Provide map/output, fleet in [1,128], and nonnegative offset/limit')
        if not all(math.isfinite(v) and v > 0 for v in (args.seconds, args.speed)):
            parser.error('seconds and speed must be finite and positive')
        if args.placement_fraction is not None and not 0 <= args.placement_fraction <= 1:
            parser.error('placement-fraction must be in [0,1]')
        if args.replay:
            replay(args)
        else:
            if not args.xodr:
                parser.error('Live audit requires --xodr for explicit source lane center t')
            audit(args)
