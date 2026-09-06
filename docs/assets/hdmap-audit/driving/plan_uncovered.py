"""Plan 20m-spaced native source placements over missing cell intervals; no VTD control."""
import bisect
import collections
import hashlib
import json
import math
from pathlib import Path
import sys
import hdmap

map_path, summary_path, output = map(Path, sys.argv[1:4])
summary = json.loads(summary_path.read_text())
digest = hashlib.sha256(map_path.read_bytes()).hexdigest()
if digest != summary['map_sha256']:
    raise ValueError('Coverage summary map SHA differs from selected map')
model = hdmap.hdmap_init(str(map_path))
covered = set(summary['moving_cells'])
by_lane = collections.defaultdict(list)
for cell in model.cells():
    by_lane[cell.parent().lanelet_id].append(cell)
targets, excluded = [], []
for lane in model.laneletMap().laneletLayer:
    attrs = lane.attributes
    road = int(attrs['xodr_road_id'])
    cells = sorted(by_lane[lane.id], key=lambda cell:cell.parent().index_in_lanelet)
    station = 0.; missing = []
    for cell in cells:
        length = float(cell.polygon3d().attributes['centerline_length_m'])
        if cell.id not in covered:
            missing.append((station, station+length, cell.id))
        station += length
    if not missing:
        continue
    if road == 6024:
        excluded.append(dict(target_lanelet=lane.id, source_road=road,
                             cell_ids=[item[2] for item in missing],
                             reason='Known native ghostdriver null-read crash at road 6024 midpoint, reproduced twice; no automatic repeat'))
        continue
    # Same sampled left/right boundaries and linear source stations as build_map.subdivide.
    left, right = list(lane.leftBound), list(lane.rightBound)
    if len(left) != len(right) or len(left)<2:
        raise ValueError(f'Unexpected source boundary sampling: {lane.id}')
    centers = [((a.x+b.x)*.5,(a.y+b.y)*.5) for a,b in zip(left,right)]
    distances = [0.]
    for a,b in zip(centers,centers[1:]):distances.append(distances[-1]+math.dist(a,b))
    begin,end = float(attrs['xodr_s_begin']),float(attrs['xodr_s_end'])
    if int(attrs['xodr_lane_id'])>0:begin,end=end,begin
    next_missing = 0
    while next_missing<len(missing):
        first = missing[next_missing][0]
        distance = max(.02,first-1.5)
        i = min(len(distances)-2,max(0,bisect.bisect_right(distances,distance)-1))
        ratio = (distance-distances[i])/(distances[i+1]-distances[i])
        source_s = begin+(end-begin)*(i+ratio)/(len(distances)-1)
        selected=[]
        while next_missing<len(missing) and missing[next_missing][0]<first+20.:
            selected.append(missing[next_missing][2]);next_missing+=1
        targets.append(dict(target_lanelet=lane.id,source_road=road,
                            source_lane=int(attrs['xodr_lane_id']),placement_s=source_s,
                            placement_centerline_distance_m=distance,planned_missing_cell_ids=selected))
result=dict(map=str(map_path),map_sha256=digest,coverage_summary=str(summary_path),
            spacing_m=20.,lead_in_m=1.5,targets=targets,excluded=excluded,
            planned_missing_cell_count=sum(len(item['planned_missing_cell_ids']) for item in targets),
            estimated_fleet64_wall_seconds=3.+math.ceil(len(targets)/64)*7.1,
            estimate_only=True,actual_driving_required=True)
output.write_text(json.dumps(result,indent=2))
print(json.dumps(dict(targets=len(targets),planned_missing_cells=result['planned_missing_cell_count'],
                      excluded_cells=sum(len(item['cell_ids']) for item in excluded),
                      estimated_fleet64_wall_seconds=result['estimated_fleet64_wall_seconds'])))
