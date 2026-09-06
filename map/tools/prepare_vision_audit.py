"""Build a road-complete source/native image atlas; image review is recorded separately."""
import argparse
import concurrent.futures
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import lanelet2.io as io
from PIL import Image, ImageDraw, ImageFont
from shapely.geometry import LineString, Polygon, box
from shapely.strtree import STRtree


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def build(args):
    inventory = json.loads(args.coverage.read_text())
    output = args.output
    for name in ('blocks', 'tiles', 'pages'):
        (output / name).mkdir(parents=True, exist_ok=True)
    report = json.loads((args.map_dir / 'build_report.json').read_text())
    assert digest(args.map_dir / 'hdmap.bin') == report['binary_sha256']
    identity = {'xodr': digest(args.xodr), 'osgb': digest(args.osgb),
                'map': report['binary_sha256'], 'renderer': digest(args.renderer),
                'coverage': digest(args.coverage), 'lod_scale': 0.001,
                'grid_origin': [-500, -1200], 'core_m': 80, 'context_m': 10,
                'metres_per_pixel': 80 / 1024, 'block_pixels': 4352}
    identity_path = output / 'inputs.json'
    if identity_path.exists():
        assert json.loads(identity_path.read_text()) == identity, 'Inputs changed; use a new atlas directory'
    identity_path.write_text(json.dumps(identity, indent=2) + '\n')
    tiles = sorted(inventory['tiles'], key=lambda t: (t['iy'], t['ix']))
    assert len({t['id'] for t in tiles}) == len(tiles)
    for t in tiles:
        assert t['id'] == f"t{t['iy']:02d}_{t['ix']:02d}"
        assert all(math.isfinite(v) for v in t['bounds_xy_m'])
    block_ids = sorted({(t['ix'] // 4, t['iy'] // 4) for t in tiles})

    def render_block(key):
        bx, by = key
        path = output / 'blocks' / f'b{by:02d}_{bx:02d}.png'
        xmin, ymin = -500 + bx * 320, -1200 + by * 320
        if not path.exists():
            env = dict(os.environ, DISPLAY=':0', XAUTHORITY='/run/user/1000/gdm/Xauthority',
                       LD_LIBRARY_PATH='/home/stier/VIRES/VTD.2025.2/Runtime/Core/IG64/lib')
            subprocess.run([str(args.renderer), str(args.osgb), str(path),
                            str(xmin - 10), str(ymin - 10), str(xmin + 330),
                            str(ymin + 330), '4352'], env=env, check=True,
                           stdout=subprocess.DEVNULL)
        with Image.open(path) as picture:
            assert picture.size == (4352, 4352), str(path)
        print(f'block {by:02d}_{bx:02d} ready', flush=True)
        return key, path

    # ponytail: two renderer processes bound GPU/RAM use; raise only after profiling.
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        blocks = dict(pool.map(render_block, block_ids))

    native = io.load(str(args.map_dir / 'hdmap.bin'), io.Origin(0, 0))
    xy = lambda p: (float(p.x), float(p.y))
    lane_records = []
    for lane in native.laneletLayer:
        lane_records.append({'id': lane.id, 'road': report['lanelets'][str(lane.id)]['xodr'][0],
                             'bounds': [[xy(p) for p in b] for b in (lane.leftBound, lane.rightBound)],
                             'shape': Polygon([xy(p) for p in lane.polygon2d()])})
    cell_records = []
    for cell in native.polygonLayer:
        if 'type' in cell.attributes and cell.attributes['type'] == 'hdmap_cell':
            points = [xy(p) for p in cell]
            cell_records.append({'id': int(cell.attributes['cell_id']), 'points': points,
                                 'shape': Polygon(points)})
    stop_records = []
    signals = {}
    for line in native.lineStringLayer:
        kind = line.attributes['type'] if 'type' in line.attributes else ''
        if kind == 'stop_line':
            points = [xy(p) for p in line]
            stop_records.append({'id': line.id, 'points': points, 'shape': LineString(points),
                                 'attrs': dict(line.attributes)})
        elif kind == 'traffic_light':
            signals[int(line.attributes['xodr_signal_id'])] = xy(line[0])
    stop_by_id = {s['id']: s for s in stop_records}
    root = ET.parse(args.xodr).getroot()
    controllers = {int(c.get('id')): [int(v.get('signalId')) for v in c.findall('control')]
                   for c in root.findall('controller')}
    link_records = {}
    for rule in report['signal_regulations']:
        stop = stop_by_id[rule['stopline']]
        center = stop['shape'].centroid.coords[0]
        controller = rule['controller']
        for signal_id in controllers[controller]:
            endpoint = signals[signal_id]
            key = (stop['id'], signal_id, controller)
            line = LineString([center, endpoint])
            link_records[key] = {'stop': stop['id'], 'signal': signal_id,
                                 'controller': controller, 'points': [center, endpoint],
                                 'distance_m': line.length, 'shape': line}
    link_records = list(link_records.values())
    groups = [lane_records, cell_records, stop_records, link_records]
    trees = [STRtree([r['shape'] for r in group]) for group in groups]
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 19)
    block_cache = {}
    manifest = []
    for n, tile in enumerate(tiles):
        ix, iy = tile['ix'], tile['iy']
        block_key = (ix // 4, iy // 4)
        if block_key not in block_cache:
            block_cache.clear()
            block_cache[block_key] = Image.open(blocks[block_key]).convert('RGB')
        left, top = (ix % 4) * 1024, (3 - iy % 4) * 1024
        source = block_cache[block_key].crop((left, top, left + 1280, top + 1280))
        xmin, ymin = -500 + ix * 80 - 10, -1200 + iy * 80 - 10
        bounds = [xmin, ymin, xmin + 100, ymin + 100]
        roi = box(*bounds)
        selected = [[group[int(j)] for j in tree.query(roi, predicate='intersects')]
                    for group, tree in zip(groups, trees)]
        lanes, cells, stops, links = selected
        overlay = source.copy()
        draw = ImageDraw.Draw(overlay, 'RGBA')
        def project(points):
            return [((x - xmin) * 12.8, (ymin + 100 - y) * 12.8) for x, y in points]
        for c in cells:
            draw.line(project(c['points'] + c['points'][:1]), fill=(255, 165, 0, 95), width=1)
        for lane in lanes:
            for boundary in lane['bounds']:
                draw.line(project(boundary), fill=(0, 235, 255, 235), width=2)
        for link in links:
            draw.line(project(link['points']), fill=(170, 70, 220, 125), width=1)
        for stop in stops:
            unresolved = stop['attrs'].get('cell_assignment') == 'unresolved'
            draw.line(project(stop['points']), fill=(255, 20, 40, 255) if unresolved else (255, 30, 230, 255), width=4)
        for signal_id, point in signals.items():
            x, y = project([point])[0]
            if 0 <= x < 1280 and 0 <= y < 1280:
                draw.ellipse((x-4, y-4, x+4, y+4), fill=(80, 255, 100, 255))
        for label, picture in [('source', source), ('overlay', overlay)]:
            picture.save(output / 'tiles' / f"{tile['id']}_{label}.jpg", quality=95, subsampling=0)
        entry = dict(tile, view_bounds_xy_m=bounds, native_lanelets=[r['id'] for r in lanes],
                     cell_count=len(cells), cell_ids=[r['id'] for r in cells],
                     stoplines=[{'id': r['id'], 'attrs': r['attrs']} for r in stops],
                     signal_links=[{k: v for k, v in r.items() if k not in ('shape', 'points')} for r in links],
                     review_status='pending')
        manifest.append(entry)
        print(f"tile {n+1}/{len(tiles)} {tile['id']} roads={len(tile['road_ids'])}", flush=True)
    pages = []
    for start in range(0, len(manifest), 2):
        pair = manifest[start:start+2]
        page = Image.new('RGB', (1536, len(pair) * 820), '#141414')
        draw = ImageDraw.Draw(page)
        for row, tile in enumerate(pair):
            roads = ','.join(map(str, tile['road_ids']))
            label = f"{tile['id']}  XODR roads: {roads}"
            draw.text((8, row*820+4), label[:118], font=font, fill='white')
            draw.text((8, row*820+28), 'SOURCE (near LOD) | right: cyan lanes, orange cells, magenta stops, purple signal links', font=font, fill='white')
            for col, mode in enumerate(('source', 'overlay')):
                with Image.open(output/'tiles'/f"{tile['id']}_{mode}.jpg") as im:
                    page.paste(im.resize((768, 768), Image.Resampling.LANCZOS), (col*768,row*820+52))
            tile['page'] = f'p{start//2:03d}.jpg'
        filename = f'p{start//2:03d}.jpg'
        page.save(output/'pages'/filename, quality=94, subsampling=0)
        pages.append({'file': filename, 'tile_ids': [t['id'] for t in pair]})
    result = {'inputs': identity, 'road_count': len(inventory['roads']),
              'tile_count': len(manifest), 'block_count': len(blocks), 'pages': pages,
              'tiles': manifest, 'coverage_source': str(args.coverage.resolve()),
              'review_status': 'pending_visual_inspection'}
    assert set(t['id'] for t in manifest) == set(t['id'] for t in inventory['tiles'])
    assert all(len(t['view_bounds_xy_m']) == 4 for t in manifest)
    (output/'manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('road_count','tile_count','block_count','review_status')}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--coverage', type=Path, required=True)
    parser.add_argument('--map-dir', type=Path, default=Path('map'))
    parser.add_argument('--xodr', type=Path, required=True)
    parser.add_argument('--osgb', type=Path, required=True)
    parser.add_argument('--renderer', type=Path, default=Path('/tmp/hdmap-render'))
    parser.add_argument('--output', type=Path, required=True)
    build(parser.parse_args())
