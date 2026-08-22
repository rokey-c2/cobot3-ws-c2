"""Spawn NVIDIA's official IW Hub Nav2 robot setup in the project world."""

import carb
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
import omni.usd

from robots.base_robot import BaseRobotAgent


_REQUIRED_TOPICS = (
    "/cmd_vel",
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
    """Return prim paths that publish/subscribe NVIDIA's Nav2 topics."""

    found = {topic: [] for topic in _REQUIRED_TOPICS}

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

        # Keep compatibility with graphs that store the complete topic string
        # in another string attribute.
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
    """Find the robot prim that owns NVIDIA's complete IW Hub Nav2 graph.

    The code intentionally discovers the prim from NVIDIA's own navigation
    scene. It does not recreate LiDAR positions, ranges, orientations, or
    publisher settings in this project.
    """

    topic_paths = _topic_node_paths(stage)
    missing = [topic for topic, paths in topic_paths.items() if not paths]

    if missing:
        raise RuntimeError(
            "NVIDIA IW Hub navigation scene is missing required ROS topics: "
            + ", ".join(missing)
        )

    # Find an articulation root that contains every required ROS graph node.
    # This keeps the entire NVIDIA robot/sensor setup under one reference.
    candidates = []

    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue

        root_path = prim.GetPath()
        contains_all_topics = True

        for paths in topic_paths.values():
            if not any(path.HasPrefix(root_path) for path in paths):
                contains_all_topics = False
                break

        if contains_all_topics:
            candidates.append(root_path)

    if candidates:
        # Prefer the deepest articulation root when nested roots exist.
        return max(candidates, key=lambda path: path.pathString.count("/"))

    # Some sample scenes author the articulation API on a child while the ROS
    # graphs live directly below the robot Xform. In that case, find the
    # smallest common ancestor of all required topic nodes, but never import
    # the entire /World scene by accident.
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
        "Could not isolate NVIDIA IW Hub navigation robot prim without "
        "importing the whole sample scene. Topic nodes: " + details
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
            "[IW HUB] reading NVIDIA IW Hub Navigation sample to locate "
            "the official robot + dual LiDAR setup"
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

        # Only the project spawn pose is overridden. All NVIDIA robot, LiDAR,
        # ROS graph, sensor range, orientation, and topic settings stay intact.
        transform = UsdGeom.Xformable(prim)
        transform.ClearXformOpOrder()
        transform.AddTranslateOp().Set(Gf.Vec3d(*self.spawn_xyz))
        transform.AddRotateXYZOp().Set(
            Gf.Vec3f(0.0, 0.0, self.spawn_yaw)
        )

        stage.Load(self.prim_path)

        carb.log_info(
            f"[IW HUB] spawned {self.name} at {self.spawn_xyz}; "
            "NVIDIA Navigation sample sensor settings unchanged"
        )

    def post_reset(self):
        stage = omni.usd.get_context().get_stage()
        robot_prim = stage.GetPrimAtPath(self.prim_path)

        if not robot_prim.IsValid():
            raise RuntimeError(
                f"IW Hub prim is missing after reset: {self.prim_path}"
            )

        topic_paths = _topic_node_paths(stage)

        for topic in _REQUIRED_TOPICS:
            paths = [
                path
                for path in topic_paths[topic]
                if path.HasPrefix(robot_prim.GetPath())
            ]
            if not paths:
                raise RuntimeError(
                    f"NVIDIA IW Hub default ROS graph missing topic: {topic}"
                )

        carb.log_info(
            "[IW HUB] NVIDIA default Nav2 graph active: "
            "/cmd_vel, /chassis/odom, front/back 2D LiDAR"
        )

    def on_physics_step(self, dt):
        del dt
