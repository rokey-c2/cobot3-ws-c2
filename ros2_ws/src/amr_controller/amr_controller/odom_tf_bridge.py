"""Convert Isaac IW Hub odometry poses into the Nav2 odom/base TF."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomTfBridge(Node):
    def __init__(self):
        super().__init__("odom_tf_bridge")

        self.declare_parameter("robot_name", "amr_a")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            f"/{self.robot_name}/odom",
            self._on_odometry,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"TF bridge: /{self.robot_name}/odom -> "
            f"{self.odom_frame} -> {self.base_frame}"
        )

    def _on_odometry(self, message):
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBridge()
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
