import bisect,collections,gzip,json,math,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import hdmap
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest

source=ET.parse(sys.argv[2]).getroot()
roads={int(r.get('id')):r for r in source.findall('road')}
model=hdmap.hdmap_init(sys.argv[1]); native=model.laneletMap()

def value(poly,s):
 return sum(float(poly.get(k,'0'))*s**i for i,k in enumerate('abcd'))
def intervals(road_id,s):
 road=roads.get(road_id)
 if road is None:return [],{}
 sections=road.findall('lanes/laneSection');section=next((a for a in reversed(sections) if float(a.get('s'))<=s+1e-5),sections[0])
 offsets=road.findall('lanes/laneOffset');offset=next((a for a in reversed(offsets) if float(a.get('s'))<=s+1e-5),None)
 t0=value(offset,s-float(offset.get('s'))) if offset is not None else 0.
 ds=s-float(section.get('s')); spans=[];types={}
 for side,sign in [('left',1),('right',-1)]:
  cur=t0
  for lane in sorted(section.findall(side+'/lane'),key=lambda l:abs(int(l.get('id')))):
   widths=lane.findall('width');p=next((a for a in reversed(widths) if float(a.get('sOffset'))<=ds+1e-5),None)
   width=max(0.,value(p,ds-float(p.get('sOffset')))) if p is not None else 0.
   end=cur+sign*width;types[int(lane.get('id'))]=lane.get('type')
   if lane.get('type')=='driving':spans.append(sorted([cur,end]))
   cur=end
 return spans,types
for directory in map(Path,sys.argv[3:]):
 byname=collections.defaultdict(list); misses=[]
 for line in gzip.open(directory/'rdb_samples.jsonl.gz','rt'):
  a=json.loads(line)
  byname[a['name']].append({'sim_time':a['sim_time'],'xyz':a['xyz']})
  if a['phase']=='drive' and (not a['lanelets'] or not a['cells']):misses.append(a)
 timelines={name:[a['sim_time'] for a in states] for name,states in byname.items()}
 grouped=collections.defaultdict(list)
 for a in misses:
  point=BasicPoint2d(*a['xyz'][:2]);distance,nearest=findNearest(native.laneletLayer,point,1)[0]
  r=a['road'];spans,types=intervals(r[0],r[2]) if r else ([],{})
  lateral=min((max(lo-r[3],r[3]-hi,0.) for lo,hi in spans),default=None) if r else None
  peer=None
  for name,states in byname.items():
   if name==a['name']:continue
   loc=bisect.bisect_left(timelines[name],a['sim_time'])
   for n in [loc-1,loc]:
    if 0<=n<len(states) and abs(states[n]['sim_time']-a['sim_time'])<=.081:
     d=math.dist(states[n]['xyz'][:2],a['xyz'][:2])
     if peer is None or d<peer['distance_m']:peer={'name':name,'distance_m':d,'sample_dt_s':states[n]['sim_time']-a['sim_time']}
  a=dict(a,nearest_driving_polygon_m=distance,nearest_lanelet=nearest.id,
         reported_source_lane_type=types.get(r[1]) if r else None,
         reported_road_driving_lateral_distance_m=lateral,nearest_logged_audit_peer=peer)
  grouped[a['target']].append(a)
 rows=[]
 for target,states in grouped.items():
  rows.append(dict(target_lanelet=target,miss_samples=len(states),
    nearest_driving_polygon_range_m=[min(a['nearest_driving_polygon_m'] for a in states),max(a['nearest_driving_polygon_m'] for a in states)],
    reported_source_lane_types=dict(collections.Counter(str(a['reported_source_lane_type']) for a in states)),
    reported_road_ids=sorted({a['road'][0] for a in states if a['road']}),
    reported_road_driving_lateral_max_m=max((a['reported_road_driving_lateral_distance_m'] for a in states if a['reported_road_driving_lateral_distance_m'] is not None),default=None),
    nearest_logged_audit_peer_m=min((a['nearest_logged_audit_peer']['distance_m'] for a in states if a['nearest_logged_audit_peer']),default=None),
    peers_under_4m=sum(bool(a['nearest_logged_audit_peer'] and a['nearest_logged_audit_peer']['distance_m']<4.) for a in states),
    first=states[0],worst=max(states,key=lambda a:a['nearest_driving_polygon_m'])))
 output=dict(map=sys.argv[1],xodr=sys.argv[2],rows=rows,note='Nearest source lateral distance uses RDB reported road s/t and original XODR widths. Peer proximity uses retained MapAudit samples within 81ms; it does not prove collisions and excludes unlogged Ego/other traffic.')
 (directory/'miss-classification.json').write_text(json.dumps(output,indent=2))
 print(directory,json.dumps([{k:r[k] for k in ['target_lanelet','miss_samples','nearest_driving_polygon_range_m','reported_source_lane_types','reported_road_ids','reported_road_driving_lateral_max_m','nearest_logged_audit_peer_m','peers_under_4m']} for r in rows]),flush=True)
