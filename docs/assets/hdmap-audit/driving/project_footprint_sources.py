import json,gzip,math,hashlib,collections,xml.etree.ElementTree as ET
from pathlib import Path
import argparse
parser=argparse.ArgumentParser(description='Independent original-XODR classification of selected footprint departures')
parser.add_argument('--cases',type=Path,required=True)
parser.add_argument('--xodr',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
import numpy as np
from scipy.integrate import quad
from scipy.optimize import brentq, minimize_scalar
from functools import lru_cache
import lanelet2.io as io
from shapely.geometry import Point, Polygon
from shapely.strtree import STRtree
source=args.xodr; roads={int(r.get('id')):r for r in ET.parse(source).getroot().findall('road')}
def num(e,k,d=0):return float(e.get(k,d))
def active(es,s,key='s'):return next((e for e in reversed(es) if num(e,key)<=s+1e-8),es[0] if es else None)
def pol(es,s,key='s'):
 e=active(es,s,key)
 if e is None:return 0.
 ds=s-num(e,key);return sum(num(e,c)*ds**i for i,c in enumerate('abcd'))
@lru_cache(maxsize=100000)
def ref(rid,s):
 gs=roads[rid].findall('planView/geometry');g=active(gs,s);u=s-num(g,'s');h=num(g,'hdg');kind=list(g)[0];k=kind.tag
 if k=='line':dx=u;dy=0.;dh=0.
 elif k=='arc':
  cur=num(kind,'curvature');dx=math.sin(cur*u)/cur;dy=(1-math.cos(cur*u))/cur;dh=cur*u
 elif k=='spiral':
  k0=num(kind,'curvStart');dk=(num(kind,'curvEnd')-k0)/num(g,'length');dx=quad(lambda t:math.cos(k0*t+.5*dk*t*t),0,u,epsabs=1e-10)[0];dy=quad(lambda t:math.sin(k0*t+.5*dk*t*t),0,u,epsabs=1e-10)[0];dh=k0*u+.5*dk*u*u
 elif k=='poly3':
  a,bb,c,d=[num(kind,k) for k in 'abcd'];length=lambda v:quad(lambda t:math.hypot(1,bb+2*c*t+3*d*t*t),0,v,epsabs=1e-10)[0]
  v=brentq(lambda v:length(v)-u,0,max(1.,u+1.),xtol=1e-12);dx=v;dy=a+bb*v+c*v*v+d*v**3;dh=math.atan(bb+2*c*v+3*d*v*v)
 else:raise ValueError(k)
 return (num(g,'x')+math.cos(h)*dx-math.sin(h)*dy,num(g,'y')+math.sin(h)*dx+math.cos(h)*dy,h+dh)
def xy_at(rid,s,t):
 x,y,h=ref(rid,s);return np.array([x-math.sin(h)*t,y+math.cos(h)*t])
def project(rid,xy,s0):
 L=num(roads[rid],'length'); lower=max(-5.,s0-15);upper=min(L+5.,s0+15)
 # Extended s is only used where first/last source geometry is line.
 if roads[rid].findall('planView/geometry')[0][0].tag!='line':lower=max(0.,lower)
 if roads[rid].findall('planView/geometry')[-1][0].tag!='line':upper=min(L,upper)
 def tangent(s):
  x,y,h=ref(rid,float(s));return (xy[0]-x)*math.cos(h)+(xy[1]-y)*math.sin(h)
 candidates=[];ss=np.linspace(lower,upper,50)
 for a,z in zip(ss[:-1],ss[1:]):
  if tangent(a)*tangent(z)<=0:candidates.append(brentq(tangent,float(a),float(z),xtol=1e-10))
 if not candidates:
  candidates=[minimize_scalar(lambda s:sum((np.array(ref(rid,float(s))[:2])-xy)**2),bounds=(lower,upper),method='bounded').x]
 s=min(candidates,key=lambda s:sum((np.array(ref(rid,s)[:2])-xy)**2));x,y,h=ref(rid,s);t=-(xy[0]-x)*math.sin(h)+(xy[1]-y)*math.cos(h)
 secs=roads[rid].findall('lanes/laneSection');sec=active(secs,max(0,min(L,s)));ds=s-num(sec,'s');offset=pol(roads[rid].findall('lanes/laneOffset'),s);intervals=[]
 for sign,side in [(1,'left'),(-1,'right')]:
  cursor=offset
  for lane in sorted(sec.findall(side+'/lane'),key=lambda l:abs(int(l.get('id')))):
   width=max(0,pol(lane.findall('width'),ds,'sOffset'));other=cursor+sign*width;intervals.append({'lane':int(lane.get('id')),'type':lane.get('type'),'t_bounds':sorted([cursor,other]),'width':width});cursor=other
 hit=[i for i in intervals if i['t_bounds'][0]-1e-7<=t<=i['t_bounds'][1]+1e-7]
 driving=[i for i in intervals if i['type']=='driving'];margin=max(min(t-i['t_bounds'][0],i['t_bounds'][1]-t) for i in driving)
 cls='outside_road_s_extent' if s < -1e-5 or s>L+1e-5 else ('inside_original_driving' if any(i['type']=='driving' for i in hit) else 'outside_original_driving')
 return {'road':rid,'raw_projected_s':s,'raw_projected_t':t,'road_length':L,'longitudinal_residual_m':tangent(s),'source_interval_hits':hit,'driving_intervals':driving,'signed_driving_lateral_margin_m':margin,'classification':cls}

from shapely import wkt, set_precision
from shapely.ops import unary_union
pilotpath=args.cases;pilot=json.loads(pilotpath.read_text());result=[]
def bounds(rid,s):
 r=roads[rid];sec=active(r.findall('lanes/laneSection'),s);ds=s-num(sec,'s');offset=pol(r.findall('lanes/laneOffset'),s);out={}
 for sign,side in [(1,'left'),(-1,'right')]:
  cursor=offset
  for lane in sorted(sec.findall(side+'/lane'),key=lambda l:abs(int(l.get('id')))):
   width=max(0,pol(lane.findall('width'),ds,'sOffset'));outer=cursor+sign*width;out[int(lane.get('id'))]=(cursor,outer,lane.get('type'));cursor=outer
 return out
for row in pilot['rows']:
 if not row['point_pass_2cm'] or row['bbox_outside_area_m2']<=.05:continue
 d=row['road'];out=wkt.loads(row['outside_wkt']);mid=np.array(row['outside_centroid']);rid=int(d[0]) if d else None
 if rid is None:result.append({'target':row['target'],'frame':row['frame'],'classification':'no_reported_road_for_source_classification'});continue
 proj=project(rid,mid,float(d[2]));s0=proj['raw_projected_s'];L=num(roads[rid],'length');lo=max(0.,s0-6);hi=min(L,s0+6);pieces=collections.defaultdict(list)
 sections=roads[rid].findall('lanes/laneSection');ss=sorted(set(np.linspace(lo,hi,max(2,math.ceil((hi-lo)/.1)+1)).tolist()+[num(sec,'s') for sec in sections if lo<num(sec,'s')<hi])) if hi>lo else []
 for a,z in zip(ss[:-1],ss[1:]):
  aa=a+1e-8;zz=z-1e-8;ba,bz=bounds(rid,aa),bounds(rid,zz)
  for lid,(left,right,typ) in ba.items():
   if typ=='driving' or lid not in bz:continue
   lz,rz,tz=bz[lid]
   if tz!=typ:continue
   p=Polygon([xy_at(rid,aa,left),xy_at(rid,zz,lz),xy_at(rid,zz,rz),xy_at(rid,aa,right)])
   if p.is_valid and p.area>1e-12:pieces[typ].append(p)
 areas={typ:float(out.intersection(set_precision(unary_union(pp),.00001)).area) for typ,pp in pieces.items()};entry={'target':row['target'],'frame':row['frame'],'reference_xyz':row['xyz'],'reported_road':d,'bbox_outside_area_m2':row['bbox_outside_area_m2'],'yaw_outside_area_m2':row['yaw_outside_area_m2'],'outside_area_after2cm_map_buffer_m2':row['bbox_outside_buffer2cm_area_m2'],'outside_centroid_source_projection':proj,'overlap_original_non_driving_area_m2':areas,'height_sensitivity_outside_areas':[row['outside_height_tolerance_0.2_m2'],row['bbox_outside_area_m2'],row['outside_height_tolerance_1.0_m2']]};result.append(entry)
out={'xodr':str(source),'xodr_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'cases':str(args.cases),'cases_sha256':hashlib.sha256(args.cases.read_bytes()).hexdigest(),'method':'For each representative point-passing moving bbox sample from the50 largest distinct reported road/lane pairs, independently project outside centroid on reported source road. Build original non-driving lane polygons with exact XML widths, independent road curve at0.1m s steps within6m of projected centroid; measure outside polygon intersection area. Source type area is XY corroboration on reported road only, not exclusive classification among overlapping junction roads; source sidewalk heights are not separately evaluated here. Driving union height filtering is in map/audit/vehicle_footprint_audit.json.','rows':result,'summary':{'additional_samples':len(result),'with_sidewalk_area_over_0_05':sum(r.get('overlap_original_non_driving_area_m2',{}).get('sidewalk',0)>.05 for r in result),'with_border_area_over_0_05':sum(r.get('overlap_original_non_driving_area_m2',{}).get('border',0)>.05 for r in result)}}
p=args.output;p.write_text(json.dumps(out,indent=2)+'\n');print(p);print(json.dumps(out['summary']))
