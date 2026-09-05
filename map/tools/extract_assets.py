import argparse
import base64
import json
import re
from pathlib import Path

from PIL import Image, ImageDraw


def extract_stopline_mesh(source, output):
    import numpy as np

    markings = {}
    triangles = np.loadtxt(source).reshape(-1, 3, 3)
    for triangle in triangles:
        for corner in range(3):
            origin = triangle[corner]
            first = triangle[(corner + 1) % 3] - origin
            second = triangle[(corner + 2) % 3] - origin
            short, long = sorted([np.linalg.norm(first[:2]), np.linalg.norm(second[:2])])
            if not (0.2 < short < 0.4 and 2. < long < 4. and abs(np.dot(first[:2], second[:2])) < 0.01 * short * long):
                continue
            corners = np.array([origin, origin + first, origin + first + second, origin + second])
            center = corners.mean(axis=0)
            markings[tuple(np.round(center, 2))] = {"center": center.tolist(), "corners": corners.tolist()}
            break
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(list(markings.values()), indent=4) + "\n")
    print(f"Extracted {len(markings)} rectangular road-marking candidates; not all are certified stoplines")


def extract(source, output):
    output.mkdir(parents=True, exist_ok=True)
    images = []
    with source.open() as stream:
        for line in stream:
            if "ClassName osg::Image" not in line:
                continue
            header = []
            for line in stream:
                header.append(line)
                if "Data 1 {" in line:
                    break
            header = "".join(header)
            name = re.search(r'FileName "([^"]+)"', header).group(1)
            width, height, depth = map(int, re.search(r"Size (\d+) (\d+) (\d+)", header).groups())
            pixel_format = int(re.search(r"PixelFormat (\d+)", header).group(1))
            encoded = []
            for line in stream:
                if "}" in line:
                    break
                encoded.append(line.strip().strip('"'))
            if pixel_format not in (6407, 6408) or depth != 1:
                continue
            mode = "RGB" if pixel_format == 6407 else "RGBA"
            data = base64.b64decode("".join(encoded))
            image = Image.frombytes(mode, (width, height), data)
            if re.search(r"Origin 0\b", header):
                image = image.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
            destination = output / (Path(name).name + ".png")
            if not destination.exists():
                image.save(destination)
                images.append({"source_texture": name, "file": destination.name, "size": [width, height]})
    (output / "textures.json").write_text(json.dumps(images, indent=4) + "\n")
    selected = [entry for entry in images if any(word in entry["source_texture"].lower()
        for word in ("roadmark", "rm_", "srfstreet", "school", "sign", "sg_"))]
    contact = Image.new("RGB", (800, 180 * ((len(selected) + 3) // 4)), "#777777")
    draw = ImageDraw.Draw(contact)
    for index, entry in enumerate(selected):
        image = Image.open(output / entry["file"]).convert("RGBA")
        image.thumbnail((190, 145))
        origin = ((index % 4) * 200, (index // 4) * 180)
        contact.paste(image, origin, image)
        draw.text((origin[0], origin[1] + 146), entry["source_texture"][:27], fill="white")
    contact.save(output / "contact.png")
    print(f"Decoded {len(images)} embedded textures; {len(selected)} on contact sheet")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("osgt", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--white-mesh", action="store_true")
    arguments = parser.parse_args()
    if arguments.white_mesh:
        extract_stopline_mesh(arguments.osgt, arguments.output)
    else:
        extract(arguments.osgt, arguments.output)
