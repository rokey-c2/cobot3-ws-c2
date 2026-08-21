"""Publish Isaac RTX LaserScan messages with the current ROS clock stamp."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanTimeRelay(Node):
    """Make RTX scans usable by Nav2 TF message filters."""

    def __init__(self):
        super().__init__("scan_time_relay")
        self.declare_parameter("input_topic", "scan")
        self.declare_parameter("output_topic", "scan_nav")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self._relay,
            qos_profile_sensor_data,
        )
        self._received_scan = False
        self.get_logger().info(
            f"restamping LaserScan messages: {input_topic} -> {output_topic}"
        )

    def _relay(self, message):
        message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)
        if not self._received_scan:
            self._received_scan = True
            self.get_logger().info(
                f"publishing fresh scans in frame {message.header.frame_id}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimeRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
