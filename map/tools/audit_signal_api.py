"""Compare 135 plugin approaches with stationary Ego API observations.

Default: write a plan only. --run requires exclusive VTD control, operation mode,
stationary Ego, and no external control commands. Teleports never count as driving.
Only this diagnostic's Ego pose/speed changes are restored; traffic time is not rewound.
"""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import select
import signal
import socket
import struct
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'config/tools'))
sys.path.insert(0, str(ROOT / 'src/sim_bridge'))
from measure_object_offsets import SCP, RDB, ENTRY, STATE
from sim_bridge_node import TcpStream, EGO, LIGHT
from drive_map_audit import ROAD

COORD = struct.Struct('<3d3fBBH')


def ego_packets(buffer, roads):
    """Read Ego's full RDB reference, velocity and actual road; retain fragments."""
    while len(buffer) >= RDB.size:
        magic, _, header, size, frame, sim_time = RDB.unpack_from(buffer)
        total = header + size
        if magic != 35712 or header < RDB.size or total > 64 * 1024 * 1024:
            raise ValueError('Invalid RDB header')
        if len(buffer) < total:
            return
        packet = bytes(buffer[:total]); del buffer[:total]
        states, cursor = [], header
        while cursor < total:
            if total - cursor < ENTRY.size:
                raise ValueError('Truncated RDB entry')
            eh, ed, width, kind, flags = ENTRY.unpack_from(packet, cursor)
            if eh < ENTRY.size or cursor + eh + ed > total or (ed and (not width or ed % width)):
                raise ValueError('Invalid RDB entry length')
            for offset in range(cursor + eh, cursor + eh + ed, width or 1):
                if kind == 5 and width >= ROAD.size:
                    road = ROAD.unpack_from(packet, offset)
                    roads[road[0]] = dict(road=road[1], lane=road[2], s=road[4], t=road[5], frame=frame)
                elif kind == 9 and width >= STATE.size:
                    state = STATE.unpack_from(packet, offset)
                    if state[4].split(b'\0')[0] != b'Ego':
                        continue
                    velocity = COORD.unpack_from(packet, offset + STATE.size) if flags & 1 and width >= STATE.size + COORD.size else None
                    if state[18] != 0 or state[20] != 0 or not all(math.isfinite(v) for v in state[11:17]):
                        raise ValueError('Ego reference must be finite inertial coordinates')
                    if velocity and not all(math.isfinite(v) for v in velocity[:6]):
                        raise ValueError('Non-finite Ego velocity')
                    states.append(dict(id=state[0], xyz=list(state[11:14]), hpr=list(state[14:17]),
                                       velocity=list(velocity) if velocity else None, frame=frame, sim_time=sim_time))
            cursor += eh + ed
        for state in states:
            state['road'] = roads.get(state['id'])
            yield state


def plan(report):
    from build_map import polynomial

    source = next(path for path in report['inputs'] if path.endswith('.xodr'))
    root = ET.parse(source).getroot()
    roads = {int(road.get('id')): road for road in root.findall('road')}
    signals = {int(sig.get('id')): (int(road.get('id')), sig) for road in roads.values() for sig in road.findall('./signals/signal')}
    controllers = {int(c.get('id')): c for c in root.findall('controller')}
    targets = []
    for road, sign, controller, green in report['plugin_selection_table']:
        end = 0. if sign > 0 else float(roads[road].get('length'))
        candidates = [(int(key), lane) for key, lane in report['lanelets'].items()
                      if lane['xodr'][0] == road and lane['xodr'][2] * sign > 0
                      and lane['s'][0] - .6 <= end <= lane['s'][1] + .6]
        validities = [(int(v.get('fromLane')), int(v.get('toLane')))
                      for c in controllers.get(controller, ET.Element('controller')).findall('control')
                      for owner, sig in [signals[int(c.get('signalId'))]] if owner == road
                      for v in sig.findall('validity')]
        candidates.sort(key=lambda item: (bool(validities) and not any(min(a,b) <= item[1]['xodr'][2] <= max(a,b) for a,b in validities), abs(item[1]['xodr'][2]), item[0]))
        target = dict(road=road, lane_sign=sign, controller=controller, green_code=green,
                      source_controller_present=controller in controllers, source_validity=validities,
                      available_lanes=sorted({v['xodr'][2] for _,v in candidates}))
        if candidates:
            identifier, lane = candidates[0]
            begin, finish = lane['s']; margin = min(5., (finish - begin) * .25)
            position = begin + margin if sign > 0 else finish - margin
            lane_id = lane['xodr'][2]
            section = roads[road].findall('lanes/laneSection')[lane['xodr'][1]]
            side = 'left' if sign > 0 else 'right'
            source_lanes = {int(v.get('id')): v for v in section.findall(side + '/lane')}
            widths = [max(0., polynomial(source_lanes[sign*i].findall('width'), position-float(section.get('s')), 'sOffset')) for i in range(1,abs(lane_id)+1)]
            lateral = polynomial(roads[road].findall('lanes/laneOffset'),position) + sign*(sum(widths[:-1]) + widths[-1]/2)
            target.update(lane=lane_id, lanelet=identifier, s=position, t=lateral)
        targets.append(target)
    return targets


def restore_command(snapshot):
    values = dict(zip(('x', 'y', 'z'), snapshot['xyz']))
    values.update(zip(('hDeg', 'pDeg', 'rDeg'), map(math.degrees, snapshot['hpr'])))
    pose = ' '.join(f'{key}="{value:.17g}"' for key, value in values.items())
    return f'<Set entity="player" name="Ego"><PosInertial {pose}/><Speed value="{snapshot["speed"]:.17g}"/></Set>'


def pose_error(actual, expected):
    return dict(position_m=math.dist(actual['xyz'], expected['xyz']),
                hpr_rad=max(abs(math.remainder(a-b, 2*math.pi)) for a,b in zip(actual['hpr'], expected['hpr'])),
                speed_mps=abs(math.dist(actual['velocity'][:3], (0.,0.,0.))-abs(expected['speed'])) if actual['velocity'] else None)


def run(args, report, targets):
    result = dict(map_binary_sha256=report['binary_sha256'], input_hashes=report['inputs'],
                  mode='stationary_ego_teleport_api_selection', driving_coverage=False,
                  requested_approaches=len(targets), results=[], restoration={'attempted': False})
    buffer, roads, latest = bytearray(), {}, None
    with socket.create_connection(('127.0.0.1', 48190), timeout=3.) as rdb, \
         socket.create_connection(('127.0.0.1', 9910), timeout=3.) as api_socket, \
         socket.create_connection(('127.0.0.1', 48179), timeout=3.) as control:
        api_socket.settimeout(.2)
        api = TcpStream(api_socket)

        def command(xml):
            data = xml.encode()
            control.sendall(SCP.pack(40108, 1, b'SignalApiAudit', b'TaskControl', len(data)) + data)

        def observe(seconds):
            nonlocal latest
            samples, finish = [], time.monotonic() + seconds
            while time.monotonic() < finish:
                ready = select.select([rdb, api_socket], [], [], .1)[0]
                if rdb in ready:
                    data = rdb.recv(4 * 1024 * 1024)
                    if not data:
                        raise ConnectionError('RDB stream closed')
                    buffer.extend(data)
                    for state in ego_packets(buffer, roads):
                        latest = state
                if api_socket in ready:
                    packet = api.receive()
                    if packet and latest:
                        pose, light = EGO.unpack_from(packet), LIGHT.unpack_from(packet, 1104)
                        if not all(math.isfinite(v) for v in pose):
                            raise ValueError('Non-finite API Ego pose')
                        samples.append(dict(ego=latest, api_pose=list(pose), controller=light[0], state=light[1],
                                            api_rdb_distance_m=math.dist(pose[:3], latest['xyz'])))
            return samples

        snapshot = None
        try:
            observe(2.)
            if latest is None or latest['velocity'] is None:
                raise RuntimeError('No full Ego RDB velocity snapshot; start VTD operation mode before --run')
            snapshot = dict(latest)
            if snapshot['velocity'][7] not in (0, 1):
                raise RuntimeError('Ego velocity frame must be inertial or player coordinates; no placement commands sent')
            speed = math.dist(snapshot['velocity'][:3], (0., 0., 0.))
            if speed > .02 or max(abs(v) for v in snapshot['velocity'][3:6]) > .02:
                raise RuntimeError('Ego must already be stationary; no placement commands sent')
            # Only a stationary snapshot is accepted; do not reconstruct arbitrary lateral dynamics.
            direction = snapshot['velocity'][0] if snapshot['velocity'][7] == 1 else sum(v*f for v,f in zip(snapshot['velocity'][:2], (math.cos(snapshot['hpr'][0]),math.sin(snapshot['hpr'][0]))))
            snapshot['speed'] = math.copysign(speed, direction)
            (args.output / 'ego_snapshot.json').write_text(json.dumps(snapshot, indent=2) + '\n')
            (args.output / 'restore_ego.xml').write_text(restore_command(snapshot) + '\n')
            result['snapshot'] = snapshot
            for index, target in enumerate(targets):
                if 'lane' not in target:
                    result['results'].append(dict(target, status='no_source_terminal_lane'))
                    continue
                command('<Set entity="player" name="Ego"><Speed value="0"/></Set>')
                command(f'<Set entity="player" name="Ego"><TrackPos track="{target["road"]}" lane="{target["lane"]}" s="{target["s"]:.17g}" t="{target["t"]:.17g}" dhDeg="{180 if target["lane"] > 0 else 0}"/></Set>')
                observe(.7)
                samples = observe(args.seconds)
                valid = [s for s in samples if s['api_rdb_distance_m'] < .05 and s['ego']['road']
                         and s['ego']['road']['frame'] >= s['ego']['frame'] - 1
                         and (s['ego']['road']['road'], s['ego']['road']['lane']) == (target['road'], target['lane'])
                         and abs(s['ego']['road']['s'] - target['s']) < .25
                         and abs(s['ego']['road']['t'] - target['t']) < .05
                         and s['ego']['velocity'] and math.dist(s['ego']['velocity'][:3], (0.,0.,0.)) < .05]
                observed = collections.Counter((s['controller'], s['state']) for s in valid)
                status = ('placement_or_stationarity_unconfirmed' if len(valid) < 3 else
                          'matching_controller' if all(c == target['controller'] for c,_ in observed) else
                          'zero_controller' if all(c == 0 for c,_ in observed) else 'wrong_or_mixed_controller')
                entry = dict(target, status=status, samples=len(samples), valid_samples=len(valid),
                             observed=[dict(controller=c, state=s, samples=n) for (c,s),n in sorted(observed.items())],
                             evidence=samples[-10:], wrong_road_lane_samples=sum(bool(s['ego']['road']) and
                                 (s['ego']['road']['road'],s['ego']['road']['lane']) != (target['road'],target['lane']) for s in samples))
                result['results'].append(entry)
                (args.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
                print(json.dumps(dict(completed=index+1, total=len(targets), road=target['road'], controller=target['controller'], status=status)), flush=True)
        finally:
            if snapshot is not None and 'speed' in snapshot:
                result['restoration']['attempted'] = True
                try:
                    command(restore_command(snapshot))
                    observations = observe(1.)
                    errors = [pose_error(s['ego'], snapshot) for s in observations[-3:]]
                    result['restoration'].update(final_errors=errors, final_observations=observations[-3:],
                        verified=len(errors)==3 and all(e['position_m'] < .02 and e['hpr_rad'] < .001 and e['speed_mps'] is not None and e['speed_mps'] < .02 for e in errors))
                except Exception as error:
                    result['restoration'].update(verified=False, error=str(error))
            result['status_counts'] = dict(collections.Counter(e['status'] for e in result['results']))
            (args.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
    if not result['restoration'].get('verified'):
        raise RuntimeError(f'Ego restoration unverified; retained snapshot and exact command in {args.output}')


def self_check():
    raw_state = STATE.pack(1,1,1,0,b'Ego',4.,2.,1.5,0.,0.,.75,10.,20.,30.,.1,.2,.3,0,0,0,0,0,1)
    velocity = COORD.pack(0.,0.,0.,0.,0.,0.,0,0,0)
    body = raw_state + velocity
    road = ROAD.pack(1,72,2,0,5.,4.5,0.,0.,0.,0.,0,0,0,0.)
    data = ENTRY.pack(ENTRY.size,len(body),len(body),9,1) + body
    data += ENTRY.pack(ENTRY.size,len(road),len(road),5,0) + road
    packet = RDB.pack(35712,1,RDB.size,len(data),7,1.2) + data
    buffer = bytearray(packet[:20]); roads={}
    assert list(ego_packets(buffer,roads)) == []
    buffer.extend(packet[20:]+packet)
    states = list(ego_packets(buffer,roads))
    assert len(states)==2 and not buffer and states[0]['road']['lane']==2
    snapshot=dict(states[0],speed=0.)
    restored=ET.fromstring(restore_command(snapshot)); pose=restored.find('PosInertial')
    assert [float(pose.get(a)) for a in ('x','y','z')]==snapshot['xyz']
    assert max(abs(math.radians(float(pose.get(a)))-v) for a,v in zip(('hDeg','pDeg','rDeg'),snapshot['hpr']))<1e-15
    assert pose_error(states[0],snapshot)['position_m']==0
    try:
        list(ego_packets(bytearray(b'\0'*RDB.size),{}))
    except ValueError:
        pass
    else:
        raise AssertionError('Malformed header accepted')
    print('PASS: fragmented RDB, same-frame road association, velocity, malformed header and XYZ/HPR restore serialization')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path,default=ROOT/'map/build_report.json')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--seconds',type=float,default=1.)
    parser.add_argument('--limit',type=int,default=0)
    parser.add_argument('--run',action='store_true')
    parser.add_argument('--self-check',action='store_true')
    args=parser.parse_args()
    if args.self_check:
        self_check()
    else:
        if not args.output or args.limit<0 or not math.isfinite(args.seconds) or args.seconds<=0:
            parser.error('Provide output, nonnegative limit, and finite positive seconds')
        if (args.output/'ego_snapshot.json').exists():
            parser.error('Output already contains an Ego snapshot; use a new evidence directory')
        report=json.loads(args.report.read_text()); targets=plan(report)
        if args.limit: targets=targets[:args.limit]
        args.output.mkdir(parents=True,exist_ok=True)
        (args.output/'plan.json').write_text(json.dumps(targets,indent=2)+'\n')
        if args.run:
            binary=args.report.parent/'hdmap.bin'
            if hashlib.sha256(binary.read_bytes()).hexdigest()!=report['binary_sha256']:
                parser.error('Map binary does not match build report')
            for path, expected in report['inputs'].items():
                if path.endswith('.xodr') or 'libHLVTD.so' in path:
                    if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=expected:
                        parser.error(f'Source/plugin changed since map generation: {path}')
            signal.signal(signal.SIGTERM, signal.default_int_handler)
            run(args,report,targets)
        else:
            print(f'Plan only: {len(targets)} approaches. No simulator connection. Use --run after simulator ownership handoff.')
