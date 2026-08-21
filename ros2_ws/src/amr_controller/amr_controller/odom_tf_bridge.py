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
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            "odom",
            self._on_odometry,
            qos_profile_sensor_data,
        )

    def _on_odometry(self, message):
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = f"{self.robot_name}/odom"
        transform.child_frame_id = f"{self.robot_name}/base_link"
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

