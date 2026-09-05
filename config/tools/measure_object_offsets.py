import argparse
import json
import math
from pathlib import Path
import select
import socket
import struct
import time

SCP = struct.Struct('<HH64s64sI')
RDB = struct.Struct('<HHIIId')
ENTRY = struct.Struct('<IIIHH')
STATE = struct.Struct('<IBBH32s6f3d3fBBHIHh')
OBJECT = struct.Struct('<I8f')


def send(command):
    data = command.encode()
    with socket.create_connection(('127.0.0.1', 48179), timeout=3.) as connection:
        connection.sendall(SCP.pack(40108, 1, b'OffsetMeasurement', b'TaskControl', len(data)) + data)


def start():
    send('<SimCtrl><Start mode="operation"/></SimCtrl>')
    with socket.create_connection(('127.0.0.1', 48190), timeout=60.) as connection:
        header = bytearray()
        while len(header) < RDB.size:
            chunk = connection.recv(RDB.size - len(header))
            if not chunk:
                raise ConnectionError('RDB closed before simulation started')
            header.extend(chunk)
        if RDB.unpack(header)[0] != 35712:
            raise ValueError('expected VTD RDB on port 48190')


def record(seconds, output):
    sockets = {socket.create_connection(('127.0.0.1', port), timeout=3.): kind for port, kind in [(48190, 'rdb'), (9910, 'api')]}
    buffers = {connection: bytearray() for connection in sockets}
    counts = {'rdb': 0, 'api': 0}
    measurements = {'rdb': {}, 'api': {}}
    rdb_history = {}
    api_history = []
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            for connection in select.select(list(sockets), [], [], 0.2)[0]:
                data = connection.recv(1024 * 1024)
                if not data:
                    raise ConnectionError('stream closed')
                buffer = buffers[connection]
                buffer.extend(data)
                kind = sockets[connection]
                if kind == 'api':
                    while len(buffer) >= 1109:
                        packet = bytes(buffer[:1109])
                        del buffer[:1109]
                        counts['api'] += 1
                        for offset in range(24, 1104, 36):
                            obj = OBJECT.unpack_from(packet, offset)
                            if any(obj):
                                measurements['api'][str(obj[0])] = obj
                                api_history.append(obj)
                else:
                    while len(buffer) >= RDB.size:
                        magic, version, header_size, data_size, frame, sim_time = RDB.unpack_from(buffer)
                        assert magic == 35712, (magic, buffer[:32].hex())
                        if len(buffer) < header_size + data_size:
                            break
                        packet = bytes(buffer[:header_size + data_size])
                        del buffer[:header_size + data_size]
                        counts['rdb'] += 1
                        cursor = header_size
                        while cursor < len(packet):
                            entry_header, entry_data, element_size, package_id, flags = ENTRY.unpack_from(packet, cursor)
                            if package_id == 9 and element_size >= STATE.size:
                                for offset in range(cursor + entry_header, cursor + entry_header + entry_data, element_size):
                                    state = STATE.unpack_from(packet, offset)
                                    name = state[4].split(b'\0')[0].decode(errors='replace')
                                    measurements['rdb'][str(state[0])] = {'name': name, 'id': state[0],
                                        'category': state[1], 'type': state[2], 'dimensions': state[5:8],
                                        'offset': state[8:11], 'reference': state[11:14], 'hpr': state[14:17],
                                        'coord_flags': state[17], 'coord_type': state[18], 'parent': state[20],
                                        'model_id': state[22], 'frame': frame, 'sim_time': sim_time}
                                    rdb_history.setdefault(state[0], []).append(measurements['rdb'][str(state[0])])
                            cursor += entry_header + entry_data
    finally:
        for connection in sockets:
            connection.close()
    matches = {}
    for identifier, history in rdb_history.items():
        poses = {struct.pack('<4f', *state['reference'], state['hpr'][0]): state for state in history}
        matched = [obj for obj in api_history if obj[0] == identifier and struct.pack('<4f', *obj[1:5]) in poses]
        matches[str(identifier)] = {'api_samples': sum(obj[0] == identifier for obj in api_history),
            'exact_float32_reference_matches': len(matched),
            'offsets_observed': sorted(set(tuple(state['offset']) for state in history)),
            'sizes_observed': sorted(set(tuple(state['dimensions']) for state in history)),
            'heading_min_max_rad': [min(state['hpr'][0] for state in history), max(state['hpr'][0] for state in history)]}
        if matched:
            sample = matched[-1]
            matches[str(identifier)]['sample'] = {'api': sample, 'rdb': poses[struct.pack('<4f', *sample[1:5])]}
    result = {'counts': counts, 'matches': matches, **measurements}
    output.write_text(json.dumps(result, indent=4) + '\n')
    print(output, counts, matches, flush=True)
    assert counts['rdb'] and counts['api'], result
    assert any(item['exact_float32_reference_matches'] for item in matches.values()), result
    return result


def export_offsets(reports, path):
    import yaml

    offsets = {}
    for report in reports:
        matched_ids = {identifier for identifier, match in report['matches'].items() if 'sample' in match}
        missing = set(report.get('api', {})) - matched_ids
        if missing:
            raise ValueError(f'no matching RDB geometry for API IDs {sorted(missing)}')
        for identifier, match in report['matches'].items():
            if 'sample' not in match:
                continue
            raw, api = match['sample']['rdb'], match['sample']['api']
            if (len(match['offsets_observed']) != 1
                    or len(match.get('sizes_observed', [raw['dimensions']])) != 1):
                raise ValueError(f'object ID {identifier} changed geometry during capture')
            if int(identifier) != raw['id'] or raw['id'] != api[0] or list(raw['dimensions']) != list(api[6:9]):
                raise ValueError(f'object ID {identifier} does not match its API sample')
            if struct.pack('<4f', *raw['reference'], raw['hpr'][0]) != struct.pack('<4f', *api[1:5]):
                raise ValueError(f'object ID {identifier} reference does not match its API sample')
            if raw['coord_type'] != 0 or raw['parent'] != 0:
                raise ValueError(f'object ID {identifier} is not in inertial coordinates')
            geometry = {'size_m': [round(value, 6) for value in raw['dimensions']],
                'center_offset_m': [round(value, 6) for value in raw['offset']]}
            previous = offsets.get(raw['id'])
            if previous is not None and previous != geometry:
                raise ValueError(f'object ID {identifier} maps to different geometry across reports')
            offsets[raw['id']] = geometry
    if not offsets:
        raise ValueError('no matched objects; existing offset file was not replaced')
    path.write_text(yaml.safe_dump(offsets, sort_keys=True, indent=4, default_flow_style=None))
    print(f'Exported {len(offsets)} object IDs to {path}', flush=True)


def check(report_path):
    import copy
    import importlib.util
    import tempfile
    import yaml

    workspace = Path(__file__).resolve().parents[2]
    specification = importlib.util.spec_from_file_location('sim_bridge', workspace / 'src/sim_bridge/sim_bridge_node.py')
    bridge = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(bridge)
    config = yaml.safe_load((workspace / 'config/runtime.yaml').read_text())
    offsets = bridge.load_object_offsets(workspace / 'config' / config['sim_bridge']['object_offsets_file'])
    phases = json.loads(report_path.read_text())['phases']
    tested = 0
    for phase in phases.values():
        for match in phase['matches'].values():
            if 'sample' not in match:
                continue
            sample = match['sample']
            api, raw = sample['api'], sample['rdb']
            assert list(struct.unpack('<4f', struct.pack('<4f', *raw['reference'], raw['hpr'][0]))) == api[1:5]
            assert raw['dimensions'] == api[6:9]
            offset = offsets[api[0]]['center_offset_m']
            assert all(abs(actual - expected) < 1e-6 for actual, expected in zip(raw['offset'], offset))
            packet = bytearray(1109)
            OBJECT.pack_into(packet, 24, *api)
            _, objects, _ = bridge.decode_frame(packet, offsets)
            heading = raw['hpr'][0]
            expected = [raw['reference'][0] + math.cos(heading) * raw['offset'][0] - math.sin(heading) * raw['offset'][1],
                raw['reference'][1] + math.sin(heading) * raw['offset'][0] + math.cos(heading) * raw['offset'][1],
                raw['reference'][2] + raw['offset'][2] - raw['dimensions'][2] / 2]
            assert len(objects) == 1
            assert max(abs(actual - target) for actual, target in zip(objects[0][1:4], expected)) < 1e-4
            for forward in (-0.5, 0.5):
                for left in (-0.5, 0.5):
                    for up in (0., 1.):
                        local_x = forward * raw['dimensions'][0]
                        local_y = left * raw['dimensions'][1]
                        corner = [objects[0][1] + math.cos(heading) * local_x - math.sin(heading) * local_y,
                            objects[0][2] + math.sin(heading) * local_x + math.cos(heading) * local_y,
                            objects[0][3] + up * raw['dimensions'][2]]
                        raw_x, raw_y = raw['offset'][0] + local_x, raw['offset'][1] + local_y
                        expected_corner = [raw['reference'][0] + math.cos(heading) * raw_x - math.sin(heading) * raw_y,
                            raw['reference'][1] + math.sin(heading) * raw_x + math.cos(heading) * raw_y,
                            raw['reference'][2] + raw['offset'][2] + (up - 0.5) * raw['dimensions'][2]]
                        assert max(abs(actual - target) for actual, target in zip(corner, expected_corner)) < 1e-4
            tested += 1
        samples = [match['sample']['api'] for match in phase['matches'].values() if 'sample' in match]
        packet = bytearray(1109)
        for index, api in enumerate(reversed(samples)):
            OBJECT.pack_into(packet, 24 + index * 36, *api)
        _, objects, _ = bridge.decode_frame(packet, offsets)
        assert [obj[0] for obj in objects] == [api[0] for api in reversed(samples)]
    assert tested >= 9, tested
    assert bridge.decode_frame(bytearray(1109), {})[1] == []
    sample = list(samples[0])
    for field, value, error in [(0, 0xffffffff, 'unmapped object ID'), (6, sample[6] + 1., 'dimensions changed')]:
        invalid = sample.copy()
        invalid[field] = value
        packet = bytearray(1109)
        OBJECT.pack_into(packet, 24, *invalid)
        try:
            bridge.decode_frame(packet, offsets)
        except ValueError as exception:
            assert error in str(exception), exception
        else:
            raise AssertionError(f'expected {error}')
    for identifier in (0, 4000000000):
        packet = bytearray(1109)
        OBJECT.pack_into(packet, 24, identifier, 10., 20., 30., math.pi / 2, 0., 4., 2., 2.)
        _, objects, _ = bridge.decode_frame(packet, {identifier: {'size_m': [4., 2., 2.], 'center_offset_m': [1., 2., 3.]}})
        assert max(abs(actual - expected) for actual, expected in zip(objects[0][1:4], [8., 21., 32.])) < 1e-5
    with tempfile.TemporaryDirectory() as directory:
        exported = Path(directory) / 'offsets.yaml'
        export_offsets(phases.values(), exported)
        assert bridge.load_object_offsets(exported) == offsets
        saved = exported.read_bytes()
        original = next(iter(phases.values()))
        conflicting = copy.deepcopy(original)
        match = next(match for match in conflicting['matches'].values() if 'sample' in match)
        match['sample']['rdb']['offset'][0] += 1.
        match['offsets_observed'] = [match['sample']['rdb']['offset']]
        missing = copy.deepcopy(original)
        missing['api'] = {'4000000000': []}
        for reports in ([original, conflicting], [missing]):
            try:
                export_offsets(reports, exported)
            except ValueError:
                assert exported.read_bytes() == saved
            else:
                raise AssertionError('invalid capture overwrote the mapping')
    print(f'PASS: {tested} mixed-model poses and 8 bbox corners; ID lookup, slot reordering, unknown/reused ID rejection, XYZ offsets')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=5.)
    parser.add_argument('--output', type=Path, default=Path('/tmp/vtd-offset-measurement.json'))
    parser.add_argument('--rotate', action='store_true')
    parser.add_argument('--check', type=Path)
    parser.add_argument('--observe', action='store_true')
    parser.add_argument('--from-report', type=Path, nargs='+')
    parser.add_argument('--export-offsets', type=Path)
    arguments = parser.parse_args()
    if arguments.check:
        check(arguments.check)
        raise SystemExit(0)
    if arguments.from_report:
        if arguments.export_offsets is None:
            parser.error('--from-report requires --export-offsets')
        reports = []
        for path in arguments.from_report:
            report = json.loads(path.read_text())
            reports.extend(report['phases'].values() if 'phases' in report else [report])
        export_offsets(reports, arguments.export_offsets)
        raise SystemExit(0)
    if not math.isfinite(arguments.seconds) or arguments.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    if arguments.observe:
        if arguments.rotate:
            parser.error('--observe never changes the scenario; do not use --rotate')
        report = record(arguments.seconds, arguments.output)
        if arguments.export_offsets:
            export_offsets([report], arguments.export_offsets)
        raise SystemExit(0)
    models = ['HyundaiIoniq6_23_White', 'BMW_Z4_2010_grey', 'SmartForTwo_14_WhiteBlack']
    reports = []
    try:
        start()
        for index, model in enumerate(models):
            send(f'<Player name="OffsetNPC{index}"><Create category="vehicle" vehicle="{model}" control="internal"/></Player>')
            time.sleep(1.)
            send(f'<Set entity="player" name="OffsetNPC{index}"><PosRelative player="Ego" dx="{15 + index * 15}" dy="0" dz="0" dhDeg="0" dpDeg="0" drDeg="0" persistent="true" override="true"/></Set>')
        time.sleep(1.)
        reports.append(record(arguments.seconds, arguments.output))
        if arguments.rotate:
            send('<Set entity="player" name="OffsetNPC0"><PosRelative player="Ego" dx="20" dy="0" dz="0" dhDeg="90" dpDeg="0" drDeg="0" persistent="true" override="true"/></Set>')
            time.sleep(1.)
            reports.append(record(arguments.seconds, arguments.output.with_stem(arguments.output.stem + '-rotated')))
            send('<Player name="OffsetNPC0"><Delete/></Player>')
            time.sleep(1.)
            send('<Player name="OffsetNPC0"><Create category="vehicle" vehicle="HyundaiIoniq6_23_White" control="internal"/></Player>')
            time.sleep(1.)
            send('<Set entity="player" name="OffsetNPC0"><PosRelative player="Ego" dx="20" dy="0" dz="0" dhDeg="180" dpDeg="0" drDeg="0" persistent="true" override="true"/></Set>')
            time.sleep(1.)
            reports.append(record(arguments.seconds, arguments.output.with_stem(arguments.output.stem + '-recreated')))
    finally:
        try:
            for index in range(3):
                send(f'<Player name="OffsetNPC{index}"><Delete/></Player>')
        finally:
            send('<SimCtrl><Stop/></SimCtrl>')
    if arguments.export_offsets:
        export_offsets(reports, arguments.export_offsets)
