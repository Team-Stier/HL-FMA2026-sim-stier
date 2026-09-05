from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .markers import MarkerOutput, bounds, line, text, xyz


class QueryDebugSink:
    def __init__(self, node, producer, enabled=True, aggregate=True, window_s=0.05):
        if window_s <= 0:
            raise ValueError("window_s must be positive")
        self.node = node
        self.enabled = enabled
        self.aggregate = aggregate
        self.window_s = window_s
        self.pending = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.output = MarkerOutput(node.create_publisher(MarkerArray, f"/debug/{producer}/cell_queries", qos))
        self.timer = node.create_timer(window_s, self.flush) if aggregate else None

    def __call__(self, operation, cells):
        if not self.enabled:
            return
        if not self.aggregate:
            self.pending.clear()
        for cell in cells:
            points = [xyz(value) for value in cell.polygon3d()]
            geometry = tuple((value.x, value.y, value.z) for value in points)
            key = operation, cell.id, geometry
            previous = self.pending.get(key)
            self.pending[key] = (points, (previous[1] if previous else 0) + 1)
        if not self.aggregate:
            self.flush()

    def flush(self):
        header = Header(stamp=self.node.get_clock().now().to_msg(), frame_id="map")
        result = []
        for index, ((operation, cell_id, geometry), (points, count)) in enumerate(self.pending.items()):
            namespace = f"query/{operation}"
            result.append(bounds(header, namespace + "/bounds", index, points, (1, 0, 1, 1)))
            result.append(line(header, namespace + "/polygon", index, points, (1, 1, 0, 1), closed=True))
            result.append(text(header, namespace + "/info", index, points[0],
                f"cell={cell_id} count={count} aggregate={self.aggregate} window={self.window_s if self.aggregate else 0:g}s (display stamp)"))
        self.output.publish(result)
        self.pending.clear()
