import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray, UVCoordinate


def point(x, y, z=0.0):
    return Point(x=float(x), y=float(y), z=float(z))


def xyz(value):
    return point(value.x, value.y, value.z)


def marker(header, namespace, index, kind, color=(0.8, 0.8, 0.8, 1.0)):
    result = Marker()
    result.header = header
    result.ns = namespace
    result.id = index
    result.type = kind
    result.pose.orientation.w = 1.0
    result.scale.x = 0.08
    result.color.r, result.color.g, result.color.b, result.color.a = map(float, color)
    return result


def line(header, namespace, index, points, color, width=0.08, closed=False, kind=Marker.LINE_STRIP):
    result = marker(header, namespace, index, kind, color)
    result.scale.x = width
    result.points = list(points)
    if closed and result.points:
        result.points.append(result.points[0])
    if kind == Marker.LINE_LIST:
        result.points = [value for pair in zip(result.points, result.points[1:]) for value in pair]
    return result


def aerial_marker(header, image_path, bounds_xy, plane_z):
    minimum_x, minimum_y, maximum_x, maximum_y = bounds_xy
    result = marker(header, "aerial/osgb_orthographic", 0, Marker.TRIANGLE_LIST, (1, 1, 1, 1))
    result.scale.x = result.scale.y = result.scale.z = 1.0
    result.texture_resource = "embedded://" + image_path.name
    result.texture.header = header
    result.texture.format = "png"
    result.texture.data = image_path.read_bytes()
    for horizontal, vertical in ((0, 0), (0, 1), (1, 1), (0, 0), (1, 1), (1, 0)):
        result.points.append(point(minimum_x + horizontal*(maximum_x-minimum_x),
                                   maximum_y - vertical*(maximum_y-minimum_y), plane_z))
        result.uv_coordinates.append(UVCoordinate(u=float(horizontal), v=float(vertical)))
    return result


def text(header, namespace, index, position, label):
    result = marker(header, namespace, index, Marker.TEXT_VIEW_FACING, (1, 1, 1, 1))
    result.pose.position = position
    result.scale.z = 0.6
    result.text = label
    return result


def arrow(header, namespace, index, position, yaw, color=(1, 0.7, 0, 1)):
    result = marker(header, namespace, index, Marker.ARROW, color)
    result.points = [position, point(position.x + math.cos(yaw), position.y + math.sin(yaw), position.z)]
    result.scale.x, result.scale.y, result.scale.z = 0.08, 0.18, 0.25
    return result


def bounds_points(points):
    lower = [min(getattr(value, axis) for value in points) for axis in ('x', 'y', 'z')]
    upper = [max(getattr(value, axis) for value in points) for axis in ('x', 'y', 'z')]
    corners = [point(upper[0] if mask & 1 else lower[0],
                     upper[1] if mask & 2 else lower[1],
                     upper[2] if mask & 4 else lower[2]) for mask in range(8)]
    result = []
    for mask in range(8):
        for bit in (1, 2, 4):
            if not mask & bit:
                result.extend((corners[mask], corners[mask | bit]))
    return result


def bounds(header, namespace, index, points, color):
    result = marker(header, namespace, index, Marker.LINE_LIST, color)
    result.points = bounds_points(points)
    return result


class MarkerOutput:
    def __init__(self, publisher):
        self.publisher = publisher

    def publish(self, markers):
        self.publisher.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)] + markers))


def search_markers(message):
    count = len(message.x)
    if any(len(values) != count for values in (message.y, message.yaw, message.parent_index)):
        raise ValueError("SearchTree array lengths differ")
    if not -1 <= message.final_node_index < count:
        raise ValueError("SearchTree final index is outside the array")
    if not all(math.isfinite(value) for values in (message.x, message.y, message.yaw) for value in values):
        raise ValueError("SearchTree has non-finite coordinates")
    checked = set()
    for start in range(count):
        current, chain = start, set()
        while current != -1 and current not in checked:
            if current < 0 or current >= count or current in chain:
                raise ValueError("SearchTree has invalid/cyclic parents")
            chain.add(current)
            current = message.parent_index[current]
        checked.update(chain)
    positions = [point(message.x[index], message.y[index]) for index in range(count)]
    edges = marker(message.header, "search_tree/edges", 0, Marker.LINE_LIST, (0.3, 0.65, 1, 0.6))
    for index, parent in enumerate(message.parent_index):
        if parent != -1:
            edges.points.extend((positions[parent], positions[index]))
    selected = []
    current = message.final_node_index
    while current != -1:
        selected.append(positions[current])
        current = message.parent_index[current]
    if not count:
        return []
    result = [edges, line(message.header, "search_tree/final", 0, selected, (1, 0.3, 0, 1), 0.16, kind=Marker.LINE_LIST)]
    result.extend(arrow(message.header, "search_tree/yaw", index, position, message.yaw[index])
                  for index, position in enumerate(positions))
    return result


def road_line(header, namespace, index, points, subtype):
    result = line(header, namespace, index, points, (0.8, 0.8, 0.8, 1))
    if subtype == "solid_solid":
        result.type = Marker.LINE_LIST
        result.points = []
        for start, end in zip(points, points[1:]):
            length = math.hypot(end.x-start.x, end.y-start.y)
            if length == 0:
                continue
            for offset in (-0.12, 0.12):
                result.points.extend(point(value.x-offset*(end.y-start.y)/length,
                                           value.y+offset*(end.x-start.x)/length, value.z)
                                     for value in (start, end))
        return result
    if "dashed" not in subtype:
        return result
    result.type = Marker.LINE_LIST
    result.points = []
    travelled = 0.0
    for start, end in zip(points, points[1:]):
        distance = math.dist((start.x, start.y, start.z), (end.x, end.y, end.z))
        offset = 0.0
        while offset < distance:
            phase = (travelled + offset) % 2.0
            step = min(distance - offset, (1.0 if phase < 1.0 else 2.0) - phase)
            if step < 1e-9:
                step = min(1e-9, distance - offset)
            if phase < 1.0:
                for value in (offset, offset + step):
                    ratio = value / distance
                    result.points.append(point(start.x + ratio * (end.x-start.x),
                                               start.y + ratio * (end.y-start.y), start.z + ratio * (end.z-start.z)))
            offset += step
        travelled += distance
    return result


def batch_lines(markers):
    groups, result = {}, []
    for item in markers:
        if item.type not in (Marker.LINE_LIST, Marker.LINE_STRIP):
            result.append(item)
            continue
        color = (item.color.r, item.color.g, item.color.b, item.color.a)
        key = (item.ns, item.scale.x, color)
        if key not in groups:
            groups[key] = marker(item.header, item.ns, len(groups), Marker.LINE_LIST, color)
            groups[key].scale.x = item.scale.x
        points = item.points if item.type == Marker.LINE_LIST else [
            value for pair in zip(item.points, item.points[1:]) for value in pair]
        groups[key].points.extend(points)
    return result + list(groups.values())
