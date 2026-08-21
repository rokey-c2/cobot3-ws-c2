"""Publish the bounded, obstacle-free map used by the IW Hub global costmap."""

import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from amr_controller.map_utils import build_occupancy_grid


class StaticMapPublisher(Node):
    """Publish only the world boundary; runtime obstacles come from LiDAR."""

    def __init__(self):
        super().__init__("static_map_publisher")

        self.declare_parameter("width_m", 40.0)
        self.declare_parameter("height_m", 40.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("origin_x", -20.0)
        self.declare_parameter("origin_y", -20.0)
        self.declare_parameter("border_m", 0.2)

        resolution = float(self.get_parameter("resolution").value)
        width_m = float(self.get_parameter("width_m").value)
        height_m = float(self.get_parameter("height_m").value)
        border_m = float(self.get_parameter("border_m").value)
        origin_x = float(self.get_parameter("origin_x").value)
        origin_y = float(self.get_parameter("origin_y").value)

        if resolution <= 0.0 or width_m <= 0.0 or height_m <= 0.0:
            raise ValueError("resolution and map dimensions must be positive")

        width_cells = int(math.ceil(width_m / resolution))
        height_cells = int(math.ceil(height_m / resolution))
        border_cells = max(1, int(math.ceil(border_m / resolution)))

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(OccupancyGrid, "map", qos)

        self.message = OccupancyGrid()
        self.message.header.frame_id = "map"
        self.message.info.resolution = resolution
        self.message.info.width = width_cells
        self.message.info.height = height_cells
        self.message.info.origin.position.x = origin_x
        self.message.info.origin.position.y = origin_y
        self.message.info.origin.orientation.w = 1.0
        self.message.data = build_occupancy_grid(
            width_cells,
            height_cells,
            border_cells,
            occupied_rectangles=[],
        )

        self.publish_map()
        self.timer = self.create_timer(2.0, self.publish_map)
        self.get_logger().info(
            f"boundary-only map: {width_m:.1f} x {height_m:.1f} m, "
            f"resolution={resolution:.2f} m; obstacles=/amr_a/scan"
        )

    def publish_map(self):
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.message.info.map_load_time = self.message.header.stamp
        self.publisher.publish(self.message)


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

