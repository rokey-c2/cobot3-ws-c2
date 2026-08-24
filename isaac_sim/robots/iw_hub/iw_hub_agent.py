"""Spawn NVIDIA's official IW Hub Nav2 robot setup in the project world."""

import carb
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
import omni.usd

from robots.base_robot import BaseRobotAgent


# These three topics are authored in NVIDIA's IW Hub navigation scene and are
# enough to identify the robot + dual-LiDAR setup. Do not require /cmd_vel
# during static USD inspection: the running sample provides the command path,
# but it is not reliably discoverable as a literal authored topic in the
# source layer.
_DISCOVERY_TOPICS = (
    "/chassis/odom",
    "/front_2d_lidar/scan",
    "/back_2d_lidar/scan",
)


def _normalized_topic(namespace, topic_name):
    """Build the ROS topic exactly as an OmniGraph namespace/topic pair."""

    topic_name = str(topic_name or "").strip()
    namespace = str(namespace or "").strip()

    if not topic_name:
        return ""

    if topic_name.startswith("/"):
        return "/" + topic_name.strip("/")

    if namespace:
        return "/" + "/".join(
            part
            for part in (
                namespace.strip("/"),
                topic_name.strip("/"),
            )
            if part
        )

    return "/" + topic_name.strip("/")


def _topic_node_paths(stage):
    """Return prim paths for NVIDIA's odom and front/back LiDAR topics."""

    found = {topic: [] for topic in _DISCOVERY_TOPICS}

    for prim in stage.Traverse():
        topic_attr = prim.GetAttribute("inputs:topicName")
        namespace_attr = prim.GetAttribute("inputs:nodeNamespace")

        if topic_attr and topic_attr.IsValid():
            try:
                topic_name = topic_attr.Get()
            except Exception:
                topic_name = ""

            try:
                namespace = (
                    namespace_attr.Get()
                    if namespace_attr and namespace_attr.IsValid()
                    else ""
                )
            except Exception:
                namespace = ""

            effective_topic = _normalized_topic(
                namespace,
                topic_name,
            )

            if effective_topic in found:
                found[effective_topic].append(prim.GetPath())
                continue

        # Some graphs store the full topic string in another string attribute.
        for attribute in prim.GetAttributes():
            try:
                value = attribute.Get()
            except Exception:
                continue

            if not isinstance(value, str):
                continue

            normalized_value = _normalized_topic("", value)
            if normalized_value in found:
                found[normalized_value].append(prim.GetPath())

    return found


def _navigation_robot_prim_path(stage):
    """Find NVIDIA's IW Hub robot + dual-LiDAR prim without changing it."""

    # Explicitly load payloads/references before inspecting the source scene.
    stage.Load()

    topic_paths = _topic_node_paths(stage)
    missing = [topic for topic, paths in topic_paths.items() if not paths]

    if missing:
        raise RuntimeError(
            "NVIDIA IW Hub navigation scene is missing required sensor topics: "
            + ", ".join(missing)
        )

    candidates = []

    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue

        root_path = prim.GetPath()
        if all(
            any(path.HasPrefix(root_path) for path in paths)
            for paths in topic_paths.values()
        ):
            candidates.append(root_path)

    if candidates:
        return max(
            candidates,
            key=lambda path: path.pathString.count("/"),
        )

    all_paths = [
        path
        for paths in topic_paths.values()
        for path in paths
    ]

    common = all_paths[0]
    while common != Sdf.Path.absoluteRootPath:
        if all(path.HasPrefix(common) for path in all_paths):
            break
        common = common.GetParentPath()

    while common != Sdf.Path.absoluteRootPath:
        prim = stage.GetPrimAtPath(common)
        if prim.IsValid() and common.pathString not in {"/", "/World"}:
            has_articulation = any(
                child.HasAPI(UsdPhysics.ArticulationRootAPI)
                for child in Usd.PrimRange(prim)
            )
            if has_articulation:
                return common
        common = common.GetParentPath()

    details = "; ".join(
        f"{topic}: {[path.pathString for path in paths]}"
        for topic, paths in topic_paths.items()
    )
    raise RuntimeError(
        "Could not isolate NVIDIA IW Hub navigation robot prim. "
        "Sensor topic nodes: " + details
    )


class IwHubAgent(BaseRobotAgent):
    """Spawn NVIDIA's IW Hub navigation robot without changing its sensors."""

    def __init__(self, cfg, world, usd_path):
        super().__init__(cfg, world)
        self.usd_path = str(usd_path)
        self.spawn_xyz = tuple(cfg["spawn_xyz"])
        self.spawn_yaw = float(cfg.get("spawn_yaw", 0.0))
        self.prim_path = f"/World/Robots/{self.name}"
        self._source_prim_path = None

    def _get_source_prim_path(self):
        if self._source_prim_path is not None:
            return self._source_prim_path

        carb.log_info(
            "[IW HUB] reading NVIDIA IW Hub Navigation sample "
            "for the official robot + dual LiDAR setup"
        )

        source_stage = Usd.Stage.Open(self.usd_path)
        if source_stage is None:
            raise RuntimeError(
                "Failed to open NVIDIA IW Hub navigation scene: "
                f"{self.usd_path}"
            )

        self._source_prim_path = _navigation_robot_prim_path(source_stage)

        carb.log_info(
            "[IW HUB] NVIDIA navigation robot prim: "
            f"{self._source_prim_path}"
        )

        return self._source_prim_path

    def setup(self):
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, "/World/Robots")

        source_prim_path = self._get_source_prim_path()

        prim = stage.DefinePrim(self.prim_path, "Xform")
        prim.GetReferences().AddReference(
            self.usd_path,
            source_prim_path,
        )

        # Only the requested spawn pose is overridden. NVIDIA's sensor type,
        # position, orientation, range and ROS publisher settings are untouched.
        transform = UsdGeom.Xformable(prim)
        transform.ClearXformOpOrder()
        transform.AddTranslateOp().Set(Gf.Vec3d(*self.spawn_xyz))
        transform.AddRotateXYZOp().Set(
            Gf.Vec3f(0.0, 0.0, self.spawn_yaw)
        )

        stage.Load(self.prim_path)

        carb.log_info(
            f"[IW HUB] spawned {self.name} at {self.spawn_xyz}; "
            "NVIDIA Navigation sensor settings unchanged"
        )

    def post_reset(self):
        stage = omni.usd.get_context().get_stage()
        robot_prim = stage.GetPrimAtPath(self.prim_path)

        if not robot_prim.IsValid():
            raise RuntimeError(
                f"IW Hub prim is missing after reset: {self.prim_path}"
            )

        topic_paths = _topic_node_paths(stage)

        for topic in _DISCOVERY_TOPICS:
            paths = [
                path
                for path in topic_paths[topic]
                if path.HasPrefix(robot_prim.GetPath())
            ]
            if not paths:
                raise RuntimeError(
                    f"NVIDIA IW Hub ROS graph missing sensor topic: {topic}"
                )

        carb.log_info(
            "[IW HUB] NVIDIA dual-LiDAR/odom graph active: "
            "/chassis/odom, /front_2d_lidar/scan, /back_2d_lidar/scan"
        )

    def on_physics_step(self, dt):
        del dt
