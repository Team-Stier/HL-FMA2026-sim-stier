"""Render retained native-motion coverage. Source ROS and hdmap workspace first."""
import json
from pathlib import Path
import sys
import hdmap
from PIL import Image, ImageDraw, ImageFont

model = hdmap.hdmap_init(sys.argv[1])
summary = json.loads(Path(sys.argv[2]).read_text())
misses = json.loads(Path(sys.argv[2]).with_name('misses.json').read_text())
lanes = list(model.laneletMap().laneletLayer)
lines = [[(p.x,p.y) for p in lane.centerline] for lane in lanes]
xy = [p for line in lines for p in line]
x0,y0 = min(p[0] for p in xy),min(p[1] for p in xy)
x1,y1 = max(p[0] for p in xy),max(p[1] for p in xy)
scale = min(1650/(x1-x0),1650/(y1-y0))
left,top = 175+(1650-(x1-x0)*scale)/2,230+(1650-(y1-y0)*scale)/2
project = lambda p:(left+(p[0]-x0)*scale,top+(y1-p[1])*scale)
canvas = Image.new('RGB',(2000,2100),'white');draw=ImageDraw.Draw(canvas)
font=lambda size:ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',size)
draw.text((90,40),'VTD native motion vs repaired HD map',font=font(46),fill='#172b4d')
draw.text((90,106),f"Direction matched: {len(summary['direction_filtered_moving_roads'])}/{len(summary['source_roads'])} roads, "
          f"{len(summary['direction_filtered_moving_lanelets'])}/{summary['source_lanelets']} lanelets, "
          f"{len(summary['direction_filtered_moving_cells'])}/{summary['source_cells']} cells",font=font(29),fill='#344563')
for line in lines:
 if len(line)>1:draw.line([project(p) for p in line],fill='#c6cbd2',width=2)
cells=model.cells()
for index in summary['direction_filtered_moving_cells']:
 points=list(cells[index].polygon3d());x=sum(p.x for p in points)/len(points);y=sum(p.y for p in points)/len(points)
 px,py=project((x,y));draw.ellipse((px-1,py-1,px+1,py+1),fill='#008f71')
for lane,line in zip(lanes,lines):
 if int(lane.attributes['xodr_road_id']) in summary['direction_filtered_uncovered_roads'] and len(line)>1:
  draw.line([project(p) for p in line],fill='#db8900',width=4)
for miss in misses:
 px,py=project(miss['xyz'][:2]);draw.ellipse((px-4,py-4,px+4,py+4),fill='#d52d2d')
for i,(label,color) in enumerate([('Road geometry','#c6cbd2'),('Forward motion cell','#008f71'),('Unverified road','#db8900'),('Containment miss','#d52d2d')]):
 x=90+i*470;draw.rectangle((x,1940,x+25,1965),fill=color);draw.text((x+35,1936),label,font=font(25),fill='#172b4d')
draw.text((90,2010),'Distributed short driving samples; complete routes and all cells are not verified.',font=font(26),fill='#344563')
canvas.save(sys.argv[3])
