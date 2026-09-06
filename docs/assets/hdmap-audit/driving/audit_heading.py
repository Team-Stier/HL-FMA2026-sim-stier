"""Check actual movement and body heading against source-matched cell travel direction."""
import bisect,collections,gzip,hashlib,json,math,sys
from pathlib import Path
import hdmap

map_path=Path(sys.argv[1]);digest=hashlib.sha256(map_path.read_bytes()).hexdigest()
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'map/tools'))
from drive_map_audit import cell_headings
model=hdmap.hdmap_init(str(map_path));headings=cell_headings(model)
parents={cell.id:(int(model.laneletMap().laneletLayer[cell.parent().lanelet_id].attributes['xodr_road_id']),
                  int(model.laneletMap().laneletLayer[cell.parent().lanelet_id].attributes['xodr_lane_id']))
         for cell in model.cells()}
angle=lambda a,b:abs(math.atan2(math.sin(a-b),math.cos(a-b)))
for directory in map(Path,sys.argv[2:]):
 summary=json.loads((directory/'summary.json').read_text())
 if summary['map_sha256']!=digest:raise ValueError('Cell IDs require the run map SHA')
 previous={};counts=collections.Counter();anomalies=[]
 for line in gzip.open(directory/'rdb_samples.jsonl.gz','rt'):
  state=json.loads(line);name=state['name']
  if state['phase']!='drive':previous.pop(name,None);continue
  old=previous.get(name);previous[name]=state
  if not old or old['target']!=state['target']:continue
  dt=state['sim_time']-old['sim_time'];step=math.dist(state['xyz'][:2],old['xyz'][:2])
  if not 0<dt<=.5 or not .01<step<=max(.5,20*dt):continue
  expected=[headings[i] for i in state['cells'] if state['road'] and parents[i]==tuple(state['road'][:2])]
  if not expected:continue
  heading=min(angle(state['hpr'][0],h) for h in expected)
  travel=math.atan2(state['xyz'][1]-old['xyz'][1],state['xyz'][0]-old['xyz'][0])
  motion=min(angle(travel,h) for h in expected)
  geometric_motion=min(angle(travel,headings[i]) for i in state['cells'])
  counts['motion_over90_all_geometric_cells']+=geometric_motion>math.pi/2
  counts['moving_source_matched_samples']+=1
  counts['body_heading_over_90deg']+=heading>math.pi/2
  counts['motion_direction_over_90deg']+=motion>math.pi/2
  if heading>math.pi/2 or motion>math.pi/2:
   anomalies.append(dict(state,heading_error_deg=math.degrees(heading),motion_error_deg=math.degrees(motion),all_geometric_motion_error_deg=math.degrees(geometric_motion),step_m=step))
 result=dict(map_sha256=digest,counts=dict(counts),anomalies=anomalies,
             note='Short native initialization rotations and reverse motion are observations, not automatically map errors.')
 (directory/'heading-audit.json').write_text(json.dumps(result,indent=2))
 print(directory,json.dumps(result['counts']),flush=True)
