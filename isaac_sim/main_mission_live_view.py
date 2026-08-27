"""Run the warehouse mission with Control Tower live camera feeds.

This wrapper intentionally leaves ``main_mission.py`` untouched. It adds the
fixed warehouse top-view publisher and keeps NVIDIA's stock IW Hub front stereo
camera active so the AMR Control page can stream the robot's own view.
"""

import carb
import numpy as np

import main_mission as mission
import omni.usd
import robots.iw_hub.iw_hub_agent as iw_hub_agent
from isaacsim.sensors.camera import Camera
from pxr import Gf, UsdGeom
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOP_VIEW_CAMERA_PATH = "/World/ControlTowerTopViewCamera"
TOP_VIEW_IMAGE_TOPIC = "/top_view/rgb"
TOP_VIEW_RESOLUTION = (1280, 720)
TOP_VIEW_PUBLISH_PERIOD_SEC = 0.10

# NVIDIA's stock IW Hub already contains this stereo camera and its ROS 2
# publisher. Keep it active for the AMR Control live feed instead of creating a
# second robot-mounted camera/render product.
AMR_CAMERA_RIG_NAME = "front_stereo_camera"
_UNUSED_CAMERA_RIG_NAMES = ("intel_realsense_r200_depth",)

# The Final_Real_Map working area is centred slightly left/below world origin.
# A 32 m high camera with this lens keeps the complete inbound -> sorter ->
# outbound line visible while leaving a small border around the warehouse.
TOP_VIEW_POSITION = Gf.Vec3d(-3.5, -2.0, 32.0)
TOP_VIEW_TARGET = Gf.Vec3d(-3.5, -2.0, 0.0)
TOP_VIEW_UP = Gf.Vec3d(0.0, 1.0, 0.0)
TOP_VIEW_FOCAL_LENGTH_MM = 28.0
TOP_VIEW_HORIZONTAL_APERTURE_MM = 36.0
TOP_VIEW_VERTICAL_APERTURE_MM = 20.25


def _configure_iw_hub_camera_rigs(robot_prim):
    """Keep the front stereo camera active and disable only unused depth rig.

    ``iw_hub_agent.py`` historically disabled both rigs for RTX performance.
    The Control Tower now consumes the front stereo RGB stream, so only the
    unused Intel depth rig should remain disabled. ``TraverseAll`` also lets
    this recover the front rig if an inactive opinion is present in the stage.
    """

    enabled = []
    disabled = []
    stage = robot_prim.GetStage()
    root_path = robot_prim.GetPath()

    for prim in stage.TraverseAll():
        if not prim.GetPath().HasPrefix(root_path):
            continue

        name = prim.GetName()

        if name == AMR_CAMERA_RIG_NAME:
            if not prim.IsActive():
                prim.SetActive(True)
            enabled.append(prim.GetPath().pathString)
            continue

        if name not in _UNUSED_CAMERA_RIG_NAMES:
            continue
        if not prim.IsActive():
            continue

        prim.SetActive(False)
        disabled.append(prim.GetPath().pathString)

    if enabled:
        carb.log_info(
            "[IW HUB][CAMERA] Control Tower AMR camera kept active: "
            + ", ".join(enabled)
        )
    else:
        carb.log_warn(
            "[IW HUB][CAMERA] front_stereo_camera was not found under "
            f"{robot_prim.GetPath()}; AMR web stream will wait for a camera topic"
        )

    if disabled:
        carb.log_info(
            "[IW HUB][CAMERA] disabled unused camera rigs: "
            + ", ".join(disabled)
        )

    # Preserve the return contract of iw_hub_agent._disable_unused_cameras().
    return disabled


def _create_top_view_camera():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac USD stage is not available")

    usd_camera = UsdGeom.Camera.Define(stage, TOP_VIEW_CAMERA_PATH)
    xform = UsdGeom.Xformable(usd_camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(TOP_VIEW_POSITION)

    look_at = Gf.Matrix4d().SetLookAt(
        TOP_VIEW_POSITION,
        TOP_VIEW_TARGET,
        TOP_VIEW_UP,
    )
    orientation = look_at.GetInverse().ExtractRotation().GetQuat()
    xform.AddOrientOp().Set(Gf.Quatf(orientation))

    usd_camera.GetFocalLengthAttr().Set(TOP_VIEW_FOCAL_LENGTH_MM)
    usd_camera.GetHorizontalApertureAttr().Set(TOP_VIEW_HORIZONTAL_APERTURE_MM)
    usd_camera.GetVerticalApertureAttr().Set(TOP_VIEW_VERTICAL_APERTURE_MM)

    camera = Camera(
        prim_path=TOP_VIEW_CAMERA_PATH,
        resolution=TOP_VIEW_RESOLUTION,
    )
    camera.initialize()
    return camera


class LiveViewAmrMissionBridge(mission.AmrMissionBridge):
    """Original mission bridge plus a lightweight overhead RGB publisher."""

    def __init__(self, agent):
        super().__init__(agent)
        self._top_view_camera = None
        self._top_view_pub = None

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
                f"{TOP_VIEW_IMAGE_TOPIC} {TOP_VIEW_RESOLUTION[0]}x{TOP_VIEW_RESOLUTION[1]} "
                f"@ ~{1.0 / TOP_VIEW_PUBLISH_PERIOD_SEC:.0f} Hz"
            )
        except Exception as error:
            # The warehouse mission must stay usable even if the optional web
            # camera cannot be created on a particular Isaac installation.
            self.get_logger().error(f"top-view camera disabled: {error}")

    def _publish_top_view(self):
        if self._top_view_camera is None or self._top_view_pub is None:
            return

        rgba = self._top_view_camera.get_rgba()
        if rgba is None or rgba.size == 0:
            return

        rgb = np.ascontiguousarray(rgba[:, :, :3])
        if rgb.mean() < 1.0:
            return

        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "control_tower_top_view"
        message.height, message.width = rgb.shape[:2]
        message.encoding = "rgb8"
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = rgb.tobytes()
        self._top_view_pub.publish(message)


def main():
    # MissionIwHubAgent resolves this module-level helper when setup() runs,
    # so replacing it here changes only this live-view launch path and leaves
    # the base/main branch implementation untouched.
    iw_hub_agent._disable_unused_cameras = _configure_iw_hub_camera_rigs
    mission.AmrMissionBridge = LiveViewAmrMissionBridge
    mission.main()


if __name__ == "__main__":
    main()
