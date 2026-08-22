"""Filter IW Hub self-reflections from the RTX 2D LaserScan."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanSelfFilter(Node):
    """Remove scan hits that fall inside the IW Hub chassis footprint."""

    def __init__(self):
        super().__init__("scan_self_filter")

        self.declare_parameter("input_topic", "/amr_a/scan")
        self.declare_parameter("output_topic", "/amr_a/scan_filtered")

        # IW Hub footprint in base_link coordinates.  These defaults match the
        # footprint currently used by the custom Nav2 configuration, with a
        # small margin so the rear chassis is not marked as a moving wall.
        self.declare_parameter("self_min_x", -1.15)
        self.declare_parameter("self_max_x", 0.35)
        self.declare_parameter("self_min_y", -0.35)
        self.declare_parameter("self_max_y", 0.35)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.self_min_x = float(self.get_parameter("self_min_x").value)
        self.self_max_x = float(self.get_parameter("self_max_x").value)
        self.self_min_y = float(self.get_parameter("self_min_y").value)
        self.self_max_y = float(self.get_parameter("self_max_y").value)

        if self.self_min_x >= self.self_max_x:
            raise ValueError("self_min_x must be smaller than self_max_x")
        if self.self_min_y >= self.self_max_y:
            raise ValueError("self_min_y must be smaller than self_max_y")

        self.publisher = self.create_publisher(
            LaserScan,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"IW Hub self filter: {self.input_topic} -> {self.output_topic}; "
            f"x=[{self.self_min_x:.2f}, {self.self_max_x:.2f}], "
            f"y=[{self.self_min_y:.2f}, {self.self_max_y:.2f}]"
        )

    def _on_scan(self, message):
        filtered = LaserScan()
        filtered.header = message.header
        filtered.angle_min = message.angle_min
        filtered.angle_max = message.angle_max
        filtered.angle_increment = message.angle_increment
        filtered.time_increment = message.time_increment
        filtered.scan_time = message.scan_time
        filtered.range_min = message.range_min
        filtered.range_max = message.range_max
        filtered.ranges = list(message.ranges)
        filtered.intensities = list(message.intensities)

        angle = message.angle_min
        for index, distance in enumerate(message.ranges):
            if math.isfinite(distance):
                point_x = distance * math.cos(angle)
                point_y = distance * math.sin(angle)
                if (
                    self.self_min_x <= point_x <= self.self_max_x
                    and self.self_min_y <= point_y <= self.self_max_y
                ):
                    filtered.ranges[index] = float("inf")
            angle += message.angle_increment

        self.publisher.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSelfFilter()
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
