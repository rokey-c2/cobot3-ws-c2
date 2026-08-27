"""Run the warehouse mission with lightweight Control Tower live camera feeds.

This wrapper leaves ``main_mission.py`` untouched. It adds two low-rate web
camera publishers and applies runtime-only vision throttles:

* warehouse top view: 960x540 at ~5 Hz
* AMR follow camera: 640x360 at ~6.7 Hz
* P3020 IN RGB source: ~5 Hz
* P3020 OUT RGB source: ~5 Hz, depth kept local inside Isaac Sim

NVIDIA's stock IW Hub stereo/depth camera rigs stay disabled. The AMR page uses
one dedicated tracking camera instead, avoiding the cost of the full stereo rig.
"""

import math
import time

import numpy as np

import main_mission as mission
import omni.usd
import rclpy
from isaacsim.sensors.camera import Camera
from pxr import Gf, UsdGeom
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


TOP_VIEW_CAMERA_PATH = "/World/ControlTowerTopViewCamera"
TOP_VIEW_IMAGE_TOPIC = "/top_view/rgb"
TOP_VIEW_RESOLUTION = (960, 540)
TOP_VIEW_PUBLISH_PERIOD_SEC = 0.20

AMR_VIEW_CAMERA_PATH = "/World/ControlTowerAmrCamera"
AMR_VIEW_IMAGE_TOPIC = "/amr_a/camera/rgb"
AMR_VIEW_RESOLUTION = (640, 360)
AMR_VIEW_PUBLISH_PERIOD_SEC = 0.15
AMR_VIEW_FORWARD_OFFSET_M = 0.45
AMR_VIEW_HEIGHT_M = 0.90
AMR_VIEW_LOOK_AHEAD_M = 4.0
AMR_VIEW_LOOK_DOWN_M = 0.22

# 60 Hz simulation 기준 12 step마다 한 장 ~= 5 Hz.
P3020_VISION_PUBLISH_INTERVAL_STEPS = 12

TOP_VIEW_POSITION = Gf.Vec3d(-3.5, -2.0, 32.0)
TOP_VIEW_TARGET = Gf.Vec3d(-3.5, -2.0, 0.0)
TOP_VIEW_UP = Gf.Vec3d(0.0, 1.0, 0.0)
TOP_VIEW_FOCAL_LENGTH_MM = 28.0
TOP_VIEW_HORIZONTAL_APERTURE_MM = 36.0
TOP_VIEW_VERTICAL_APERTURE_MM = 20.25

AMR_VIEW_FOCAL_LENGTH_MM = 20.0
AMR_VIEW_HORIZONTAL_APERTURE_MM = 36.0
AMR_VIEW_VERTICAL_APERTURE_MM = 20.25


def _look_at_orientation(position, target, up):
    look_at = Gf.Matrix4d().SetLookAt(position, target, up)
    return Gf.Quatf(look_at.GetInverse().ExtractRotation().GetQuat())


def _define_camera(
    prim_path,
    resolution,
    focal_length_mm,
    horizontal_aperture_mm,
    vertical_aperture_mm,
):
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac USD stage is not available")

    usd_camera = UsdGeom.Camera.Define(stage, prim_path)
    xform = UsdGeom.Xformable(usd_camera.GetPrim())
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp()
    orient_op = xform.AddOrientOp()

    usd_camera.GetFocalLengthAttr().Set(focal_length_mm)
    usd_camera.GetHorizontalApertureAttr().Set(horizontal_aperture_mm)
    usd_camera.GetVerticalApertureAttr().Set(vertical_aperture_mm)

    camera = Camera(
        prim_path=prim_path,
        resolution=resolution,
    )
    camera.initialize()
    return camera, translate_op, orient_op


def _create_top_view_camera():
    camera, translate_op, orient_op = _define_camera(
        TOP_VIEW_CAMERA_PATH,
        TOP_VIEW_RESOLUTION,
        TOP_VIEW_FOCAL_LENGTH_MM,
        TOP_VIEW_HORIZONTAL_APERTURE_MM,
        TOP_VIEW_VERTICAL_APERTURE_MM,
    )
    translate_op.Set(TOP_VIEW_POSITION)
    orient_op.Set(
        _look_at_orientation(
            TOP_VIEW_POSITION,
            TOP_VIEW_TARGET,
            TOP_VIEW_UP,
        )
    )
    return camera


class OptimizedP3020OutRosBridge(mission.P3020OutRosBridge):
    """Keep P3020 OUT depth local and cap RGB ROS traffic."""

    def __init__(self, node):
        super().__init__(node)

        if self.depth_pub is not None:
            self._node.destroy_publisher(self.depth_pub)
            self.depth_pub = None

        self._last_rgb_publish_at = 0.0
        self._rgb_publish_interval = 1.0 / 5.0

        self._node.get_logger().info(
            "P3020 OUT vision optimization: RGB <=5 Hz, /arm_b/depth disabled"
        )

    def publish_image(self, rgba):
        now = time.monotonic()
        if now - self._last_rgb_publish_at < self._rgb_publish_interval:
            return
        self._last_rgb_publish_at = now
        super().publish_image(rgba)

    def publish_depth(self, depth_map):
        del depth_map


class OptimizedP3020OutAgent(mission.P3020UnloadToBinAgent):
    """Sample the outbound camera/depth only at the rate YOLO can consume."""

    def _wait_for_detection(self, ros_node, timeout_steps, tick_others, dt):
        not_before = ros_node.get_clock().now()
        depth_map = None
        last_frame = None

        for step in range(timeout_steps):
            if step % P3020_VISION_PUBLISH_INTERVAL_STEPS == 0:
                frame = self.camera.get_frame()
                if frame is not None:
                    last_frame = frame
                    ros_node.publish_image(frame)
                    # Depth is needed only for local pixel -> world conversion.
                    depth_map = self.camera.get_depth()

            rclpy.spin_once(ros_node._node, timeout_sec=0.0)
            pixel = ros_node.take_pixel_after(not_before)

            if pixel is not None and depth_map is not None:
                world_xyz = mission.pixel_to_world_xy(
                    pixel,
                    depth_map,
                    self.camera,
                    last_frame,
                )
                if world_xyz is not None:
                    return world_xyz

            if tick_others:
                tick_others(dt)
            self.world.step(render=True)

        return None


class LiveViewAmrMissionBridge(mission.AmrMissionBridge):
    """Original mission bridge plus two rate-limited RGB publishers."""

    def __init__(self, agent):
        super().__init__(agent)

        self._top_view_camera = None
        self._top_view_pub = None
        self._amr_view_camera = None
        self._amr_view_pub = None
        self._amr_view_translate_op = None
        self._amr_view_orient_op = None

        try:
            self._top_view_camera = _create_top_view_camera()
            self._top_view_pub = self.create_publisher(
                Image,
                TOP_VIEW_IMAGE_TOPIC,
                qos_profile_sensor_data,
            )
            self.create_timer(
                TOP_VIEW_PUBLISH_PERIOD_SEC,
                self._publish_top_view,
            )
            self.get_logger().info(
                "Control Tower top view ready: "
                f"{TOP_VIEW_IMAGE_TOPIC} "
                f"{TOP_VIEW_RESOLUTION[0]}x{TOP_VIEW_RESOLUTION[1]} "
                f"@ ~{1.0 / TOP_VIEW_PUBLISH_PERIOD_SEC:.1f} Hz"
            )
        except Exception as error:
            self.get_logger().error(f"top-view camera disabled: {error}")

        try:
            (
                self._amr_view_camera,
                self._amr_view_translate_op,
                self._amr_view_orient_op,
            ) = _define_camera(
                AMR_VIEW_CAMERA_PATH,
                AMR_VIEW_RESOLUTION,
                AMR_VIEW_FOCAL_LENGTH_MM,
                AMR_VIEW_HORIZONTAL_APERTURE_MM,
                AMR_VIEW_VERTICAL_APERTURE_MM,
            )
            self._amr_view_pub = self.create_publisher(
                Image,
                AMR_VIEW_IMAGE_TOPIC,
                qos_profile_sensor_data,
            )
            self._update_amr_camera_pose()
            self.create_timer(
                AMR_VIEW_PUBLISH_PERIOD_SEC,
                self._publish_amr_view,
            )
            self.get_logger().info(
                "Control Tower AMR camera ready: "
                f"{AMR_VIEW_IMAGE_TOPIC} "
                f"{AMR_VIEW_RESOLUTION[0]}x{AMR_VIEW_RESOLUTION[1]} "
                f"@ ~{1.0 / AMR_VIEW_PUBLISH_PERIOD_SEC:.1f} Hz"
            )
        except Exception as error:
            self.get_logger().error(f"AMR web camera disabled: {error}")

    def _publish_rgb(self, camera, publisher, frame_id):
        if camera is None or publisher is None:
            return

        rgba = camera.get_rgba()
        if rgba is None or rgba.size == 0:
            return

        rgb = np.ascontiguousarray(rgba[:, :, :3])
        if rgb.mean() < 1.0:
            return

        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.height, message.width = rgb.shape[:2]
        message.encoding = "rgb8"
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = rgb.tobytes()
        publisher.publish(message)

    def _publish_top_view(self):
        self._publish_rgb(
            self._top_view_camera,
            self._top_view_pub,
            "control_tower_top_view",
        )

    def _update_amr_camera_pose(self):
        if (
            self.agent.robot is None
            or self._amr_view_translate_op is None
            or self._amr_view_orient_op is None
        ):
            return False

        position, quaternion = self.agent.robot.get_world_pose()
        w, x, y, z = [float(value) for value in quaternion]
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)

        camera_position = Gf.Vec3d(
            float(position[0]) + AMR_VIEW_FORWARD_OFFSET_M * forward_x,
            float(position[1]) + AMR_VIEW_FORWARD_OFFSET_M * forward_y,
            float(position[2]) + AMR_VIEW_HEIGHT_M,
        )
        camera_target = Gf.Vec3d(
            camera_position[0] + AMR_VIEW_LOOK_AHEAD_M * forward_x,
            camera_position[1] + AMR_VIEW_LOOK_AHEAD_M * forward_y,
            camera_position[2] - AMR_VIEW_LOOK_DOWN_M,
        )

        self._amr_view_translate_op.Set(camera_position)
        self._amr_view_orient_op.Set(
            _look_at_orientation(
                camera_position,
                camera_target,
                Gf.Vec3d(0.0, 0.0, 1.0),
            )
        )
        return True

    def _publish_amr_view(self):
        if not self._update_amr_camera_pose():
            return
        self._publish_rgb(
            self._amr_view_camera,
            self._amr_view_pub,
            "amr_a_control_tower_camera",
        )


def main():
    # Runtime-only patches keep main_mission.py unchanged.
    mission.VISION_RGB_PUBLISH_INTERVAL_STEPS = (
        P3020_VISION_PUBLISH_INTERVAL_STEPS
    )
    mission.P3020OutRosBridge = OptimizedP3020OutRosBridge
    mission.P3020UnloadToBinAgent = OptimizedP3020OutAgent
    mission.AmrMissionBridge = LiveViewAmrMissionBridge
    mission.main()


if __name__ == "__main__":
    main()
