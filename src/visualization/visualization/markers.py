import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


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


def line(header, namespace, index, points, color, width=0.08, closed=False):
    result = marker(header, namespace, index, Marker.LINE_STRIP, color)
    result.scale.x = width
    result.points = list(points)
    if closed and result.points:
        result.points.append(result.points[0])
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


def bounds(header, namespace, index, points, color):
    lower = [min(getattr(value, axis) for value in points) for axis in ('x', 'y', 'z')]
    upper = [max(getattr(value, axis) for value in points) for axis in ('x', 'y', 'z')]
    corners = [point(upper[0] if mask & 1 else lower[0],
                     upper[1] if mask & 2 else lower[1],
                     upper[2] if mask & 4 else lower[2]) for mask in range(8)]
    result = marker(header, namespace, index, Marker.LINE_LIST, color)
    for mask in range(8):
        for bit in (1, 2, 4):
            if not mask & bit:
                result.points.extend((corners[mask], corners[mask | bit]))
    return result


class MarkerOutput:
    def __init__(self, publisher):
        self.publisher = publisher
        self.previous = set()

    def publish(self, markers):
        current = {(item.ns, item.id) for item in markers}
        removed = []
        for namespace, index in self.previous - current:
            item = Marker(ns=namespace, id=index, action=Marker.DELETE)
            removed.append(item)
        self.publisher.publish(MarkerArray(markers=markers + removed))
        self.previous.update(current)


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
    result = [edges, line(message.header, "search_tree/final", 0, selected, (1, 0.3, 0, 1), 0.16)]
    result.extend(arrow(message.header, "search_tree/yaw", index, position, message.yaw[index])
                  for index, position in enumerate(positions))
    return result


def road_line(header, namespace, index, points, subtype):
    result = line(header, namespace, index, points, (0.8, 0.8, 0.8, 1))
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
