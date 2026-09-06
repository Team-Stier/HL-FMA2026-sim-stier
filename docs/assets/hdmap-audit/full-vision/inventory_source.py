"""Read-only XODR road/80m-tile inventory. Reuses the production source geometry helpers."""
import argparse
import collections
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from shapely import make_valid
from shapely.geometry import LineString, Polygon, box
from shapely.ops import unary_union

parser = argparse.ArgumentParser()
parser.add_argument('--repo', type=Path, default=Path('/home/stier/HL-FMA2026-sim-stier'))
parser.add_argument('--xodr', type=Path, default=Path('/home/stier/vtd 자료/HL_FMA_VTD_LivingLab.xodr'))
parser.add_argument('--output', type=Path, default=Path('/tmp/hdmap-audit/full-vision/coverage-source.json'))
args = parser.parse_args()
sys.path.insert(0, str(args.repo / 'map/tools'))
from build_map import Builder, number, parse_opendrive, polynomial

# Read raw source without applying production lane/width/stop overrides.
b = Builder.__new__(Builder)
b.roads = {int(r.get('id')): r for r in ET.parse(args.xodr).getroot().findall('road')}
b.parsed = {r.id: r for r in parse_opendrive(args.xodr).roads}
b.reference_cache = {}
b.poly3_cache = {}
b.width_clamps = {}
origin = (-500., -1200.)
tile_size, grid_count, margin, spacing = 80., 32, 10., 1.
grid = box(origin[0], origin[1], origin[0] + tile_size * grid_count, origin[1] + tile_size * grid_count)
roads = []
tiles = {}
all_bounds = []
max_gap = 0.
all_samples = 0
source_lane_types = collections.Counter()
for rid, road in sorted(b.roads.items()):
    length = number(road, 'length')
    sections = road.findall('lanes/laneSection')
    reference_only = not sections
    if reference_only:
        sections = [ET.Element('laneSection', {'s': '0'})]
    strips, refs, xyz, section_rows = [], [], [], []
    geom_breaks = [number(g, 's') for g in road.findall('planView/geometry')]
    for si, sec in enumerate(sections):
        begin = number(sec, 's')
        end = number(sections[si + 1], 's') if si + 1 < len(sections) else length
        positions = set(np.linspace(begin, end, max(1, math.ceil((end-begin)/spacing)) + 1))
        positions.update(g for g in geom_breaks if begin < g < end)
        for lane in sec.findall('./*/lane'):
            if int(lane.get('id')) == 0:
                continue
            positions.update(begin + number(w, 'sOffset') for w in lane.findall('width') if begin < begin + number(w, 'sOffset') < end)
        positions = sorted(positions)
        max_gap = max(max_gap, max(np.diff(positions), default=0.))
        all_samples += len(positions)
        lanes = [l for l in sec.findall('./*/lane') if int(l.get('id')) != 0]
        left_ids = [int(l.get('id')) for l in lanes if int(l.get('id')) > 0]
        right_ids = [int(l.get('id')) for l in lanes if int(l.get('id')) < 0]
        outer_ids = [max(left_ids) if left_ids else None, min(right_ids) if right_ids else None]
        left, right, reference = [], [], []
        for s in positions:
            offset = polynomial(road.findall('lanes/laneOffset'), s)
            lateral = [b.offsets(rid, sec, lid, s)[1] if lid is not None else offset for lid in outer_ids]
            l = b.road_point(rid, s, lateral[0]); r = b.road_point(rid, s, lateral[1])
            center = b.road_point(rid, s, 0.)
            left.append(l); right.append(r); reference.append(center)
            xyz.extend((l, r, center))
        refs.append(LineString([p[:2] for p in reference]))
        for i in range(len(positions)-1):
            q = Polygon([left[i][:2], left[i+1][:2], right[i+1][:2], right[i][:2]])
            if q.area > 1e-12:
                strips.append(q if q.is_valid else make_valid(q))
        types = collections.Counter(l.get('type') for l in lanes)
        source_lane_types.update(types)
        section_rows.append({'reference_only': reference_only, 'index': si, 's': [begin, end], 'sample_count': len(positions),
            'lanes': [{'id': int(l.get('id')), 'type': l.get('type')} for l in lanes],
            'lane_types': dict(types), 'has_driving': any(l.get('type') == 'driving' for l in lanes)})
    centerline = unary_union(refs)
    surface = unary_union(strips) if strips else centerline
    context = surface.buffer(margin)
    pts = np.array(xyz)
    bounds = [float(pts[:,0].min()), float(pts[:,1].min()), float(pts[:,0].max()), float(pts[:,1].max())]
    all_bounds.append(bounds)
    indices = [range(math.floor((context.bounds[d]-origin[d])/tile_size), math.floor((context.bounds[d+2]-origin[d])/tile_size)+1) for d in (0,1)]
    touched = []
    for iy in indices[1]:
        for ix in indices[0]:
            tb = [origin[0]+ix*tile_size, origin[1]+iy*tile_size, origin[0]+(ix+1)*tile_size, origin[1]+(iy+1)*tile_size]
            tile = box(*tb)
            if not context.intersects(tile):
                continue
            tid = f't{iy:02d}_{ix:02d}'
            touched.append(tid)
            if tid not in tiles:
                tiles[tid] = {'id': tid, 'ix': ix, 'iy': iy, 'bounds_xy_m': tb,
                    'context_bounds': [tb[0]-margin,tb[1]-margin,tb[2]+margin,tb[3]+margin],
                    'block_id': f'b{iy//4:02d}_{ix//4:02d}', 'road_ids': [], 'roads_intersecting_core': []}
            tiles[tid]['road_ids'].append(rid)
            if surface.intersects(tile):
                tiles[tid]['roads_intersecting_core'].append(rid)
    roads.append({'id': rid, 'name': road.get('name'), 'junction': int(road.get('junction', '-1')),
        'length_m': length, 'reference_only': reference_only, 'has_driving': any(s['has_driving'] for s in section_rows),
        'lane_sections': section_rows, 'bounds': bounds, 'z_range': [float(pts[:,2].min()),float(pts[:,2].max())],
        'reference_bounds': list(centerline.bounds), 'context_bounds': list(context.bounds),
        'tile_ids': touched, 'source_surface_area_m2': surface.area,
        'outside_grid_area_m2': surface.difference(grid).area, 'context_outside_grid_area_m2': context.difference(grid).area})
    if len(roads) % 100 == 0:
        print(f'processed {len(roads)}/{len(b.roads)} roads', flush=True)
assert len(roads) == len(b.roads) == 651
assert all(r['tile_ids'] for r in roads)
assert max_gap <= spacing + 1e-9
non_driving = [r['id'] for r in roads if not r['has_driving']]
assert non_driving == [1251,1253,1260,1272,4741,4744,6028,6029], non_driving
bounds = np.array(all_bounds)
result = {'provenance': {'source': str(args.xodr), 'source_sha256': hashlib.sha256(args.xodr.read_bytes()).hexdigest(),
    'helper': str(args.repo / 'map/tools/build_map.py'), 'helper_sha256': hashlib.sha256((args.repo/'map/tools/build_map.py').read_bytes()).hexdigest(),
    'script': str(Path(__file__).resolve()), 'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
    'method': 'Raw XODR; no config overrides. Original reference/road_point/offsets helpers, laneSection/geometry/width breakpoints plus <=1m samples. All lane types included in outer bounds. Three roads without source laneSections are covered by reference line +10m and marked reference_only. Sampled surface strips are buffered by 10m before 80m tile selection. Curved edges approximated at <=1m; not an exact analytic boundary-extrema proof.',
    'grid': {'origin': list(origin), 'tile_size_m': tile_size, 'count': [grid_count,grid_count], 'bounds': list(grid.bounds),
        'y_direction': 'north-positive', 'tile_id': 'tYY_XX', 'context_margin_m': margin, 'block_tiles': [4,4], 'block_size_m':320},
    'counts': {'roads':len(roads), 'driving_roads':len(roads)-len(non_driving), 'non_driving_roads':len(non_driving),
        'non_driving_road_ids':non_driving,'lane_sections':sum(len(r['lane_sections']) for r in roads if not r['reference_only']),
        'source_length_m':sum(r['length_m'] for r in roads),'sample_count':all_samples,'max_sample_gap_m':max_gap,
        'tiles':len(tiles), 'blocks':len(set(t['block_id'] for t in tiles.values())), 'lane_section_lane_types':dict(source_lane_types),
        'outside_grid_road_ids':[r['id'] for r in roads if r['outside_grid_area_m2']>1e-8],
        'outside_grid_context_road_ids':[r['id'] for r in roads if r['context_outside_grid_area_m2']>1e-8]},
    'all_road_bounds': [float(bounds[:,0].min()),float(bounds[:,1].min()),float(bounds[:,2].max()),float(bounds[:,3].max())],
    'roads':roads, 'tiles':sorted(tiles.values(),key=lambda t:(t['iy'],t['ix']))}
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps(result['counts'],indent=2),flush=True)
