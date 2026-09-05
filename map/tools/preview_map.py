import argparse
from pathlib import Path

import lanelet2.core
import lanelet2.io as llio
from PIL import Image, ImageDraw


def preview(arguments):
    native_map = llio.load(str(arguments.map), llio.Origin(0., 0.))
    picture = Image.open(arguments.background).convert("RGB")
    draw = ImageDraw.Draw(picture)
    xmin, ymin, xmax, ymax = arguments.bounds

    def project(point):
        return ((point.x - xmin) / (xmax - xmin) * picture.width,
            (ymax - point.y) / (ymax - ymin) * picture.height)

    for lane in native_map.laneletLayer:
        for boundary in (lane.leftBound, lane.rightBound):
            draw.line([project(point) for point in boundary], fill="#00bbdd", width=1)
    if arguments.cells:
        for polygon in native_map.polygonLayer:
            if "type" in polygon.attributes and polygon.attributes["type"] == "hdmap_cell":
                points = [project(point) for point in polygon]
                if any(0 <= horizontal <= picture.width and 0 <= vertical <= picture.height for horizontal, vertical in points):
                    draw.line(points + points[:1], fill="#ffaa00", width=1)
    for line in native_map.lineStringLayer:
        if "type" in line.attributes and line.attributes["type"] == "stop_line":
            color = "#ff0000" if line.attributes["cell_assignment"] == "unresolved" else "#ff00ff"
            draw.line([project(point) for point in line], fill=color, width=3)
    draw.rectangle((0, 0, 470, 47), fill="black")
    draw.text((8, 5), "+X right, +Y up | cyan: lane bounds | orange: cells", fill="white")
    draw.text((8, 23), "magenta: assigned stop | red: unassigned XODR stop", fill="white")
    picture.save(arguments.output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map", type=Path)
    parser.add_argument("background", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--bounds", nargs=4, type=float, required=True)
    parser.add_argument("--cells", action="store_true")
    preview(parser.parse_args())
