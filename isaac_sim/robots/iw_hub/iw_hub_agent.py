"""Spawn NVIDIA's official IW Hub Nav2 robot setup in the project world."""

import carb
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
import omni.usd

from robots.base_robot import BaseRobotAgent


_DISCOVERY_TOPICS = (
    "/chassis/odom",
    "/front_2d_lidar/scan",
    "/back_2d_lidar/scan",
)

_LIDAR_NEAR_RANGE_ATTR = "omni:sensor:Core:nearRangeM"
_LIDAR_FAR_RANGE_ATTR = "omni:sensor:Core:farRangeM"
_LIDAR_SCAN_RATE_ATTR = "omni:sensor:Core:scanRateBaseHz"
_LIDAR_FIRING_RATE_ATTR = "omni:sensor:Core:patternFiringRateHz"


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

            effective_topic = _normalized_topic(namespace, topic_name)

            if effective_topic in found:
                found[effective_topic].append(prim.GetPath())
                continue

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
    """Find NVIDIA's IW Hub robot + dual-LiDAR prim."""

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

    all_paths = [path for paths in topic_paths.values() for path in paths]

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


def _safe_numeric_set(attribute, value):
    """Safely set a numeric USD attribute without crashing the simulation."""

    if not attribute or not attribute.IsValid():
        return False

    try:
        type_name = str(attribute.GetTypeName()).lower()

        if "int" in type_name or "uint" in type_name:
            attribute.Set(int(round(value)))
        else:
            attribute.Set(float(value))

        return True
    except Exception as exc:
        carb.log_warn(
            f"[IW HUB][LiDAR] failed to set {attribute.GetPath()}: {exc}"
        )
        return False


def _configure_lidar_range(robot_prim, min_range_m, max_range_m):
    """
    Limit the LiDAR valid range without moving the NVIDIA sensors.

    The far range is capped for performance. The near range is raised just
    enough to suppress self-returns from the cargo pod/legs while it is carried
    above the IW Hub. Scan rate, firing rate, tick rate, horizontal resolution,
    position, and orientation remain exactly as authored by NVIDIA.
    """

    if min_range_m < 0.0:
        raise ValueError("lidar_min_range_m must be >= 0")
    if max_range_m <= 0.0:
        raise ValueError("lidar_max_range_m must be > 0")
    if min_range_m >= max_range_m:
        raise ValueError("lidar_min_range_m must be smaller than lidar_max_range_m")

    configured = []
    lidar_like_paths = []

    for lidar_prim in Usd.PrimRange(robot_prim):
        near_range_attr = lidar_prim.GetAttribute(_LIDAR_NEAR_RANGE_ATTR)
        far_range_attr = lidar_prim.GetAttribute(_LIDAR_FAR_RANGE_ATTR)
        scan_rate_attr = lidar_prim.GetAttribute(_LIDAR_SCAN_RATE_ATTR)
        firing_rate_attr = lidar_prim.GetAttribute(_LIDAR_FIRING_RATE_ATTR)

        has_near_range = bool(near_range_attr and near_range_attr.IsValid())
        has_far_range = bool(far_range_attr and far_range_attr.IsValid())
        has_scan_rate = bool(scan_rate_attr and scan_rate_attr.IsValid())
        has_firing_rate = bool(
            firing_rate_attr and firing_rate_attr.IsValid()
        )

        # Do not depend on GetTypeName() == "OmniLidar".
        # Detect the NVIDIA LiDAR using its actual sensor attributes.
        if not (has_far_range and (has_scan_rate or has_firing_rate)):
            path_lower = lidar_prim.GetPath().pathString.lower()
            if "lidar" in path_lower:
                lidar_like_paths.append(
                    f"{lidar_prim.GetPath()} "
                    f"(type={lidar_prim.GetTypeName()})"
                )
            continue

        try:
            old_near_range = near_range_attr.Get() if has_near_range else None
        except Exception:
            old_near_range = None

        try:
            old_far_range = far_range_attr.Get()
        except Exception:
            old_far_range = None

        try:
            original_scan_rate = (
                scan_rate_attr.Get() if has_scan_rate else None
            )
        except Exception:
            original_scan_rate = None

        try:
            original_firing_rate = (
                firing_rate_attr.Get() if has_firing_rate else None
            )
        except Exception:
            original_firing_rate = None

        near_ok = True
        if has_near_range:
            near_ok = _safe_numeric_set(near_range_attr, min_range_m)
        else:
            carb.log_warn(
                "[IW HUB][LiDAR] near-range attribute not found on "
                f"{lidar_prim.GetPath()}; self-return suppression skipped "
                "for this sensor."
            )

        far_ok = _safe_numeric_set(far_range_attr, max_range_m)
        if not far_ok:
            carb.log_warn(
                "[IW HUB][LiDAR] sensor candidate found but "
                "far range could not be changed: "
                f"{lidar_prim.GetPath()}"
            )
            continue

        configured.append(lidar_prim.GetPath().pathString)

        carb.log_info(
            "[IW HUB][LiDAR] "
            f"{lidar_prim.GetPath()} "
            f"(type={lidar_prim.GetTypeName()}): "
            f"near {old_near_range} -> "
            f"{min_range_m if has_near_range and near_ok else old_near_range} m; "
            f"far {old_far_range} -> {max_range_m} m; "
            f"scan rate kept at NVIDIA original={original_scan_rate}; "
            f"firing rate kept at NVIDIA original={original_firing_rate}"
        )

    # LiDAR discovery failure must not terminate Isaac Sim.
    if not configured:
        carb.log_warn(
            "[IW HUB][LiDAR] No editable LiDAR range attribute was found "
            "under the referenced IW Hub."
        )
        carb.log_warn(
            "[IW HUB][LiDAR] IW Hub will continue with NVIDIA's "
            "original LiDAR settings."
        )

        if lidar_like_paths:
            carb.log_warn(
                "[IW HUB][LiDAR] LiDAR-like prims found: "
                + "; ".join(lidar_like_paths)
            )

        return []

    if len(configured) < 2:
        carb.log_warn(
            "[IW HUB][LiDAR] Expected front/back LiDARs, "
            f"but only {len(configured)} sensor(s) had their range changed: "
            f"{configured}"
        )
        carb.log_warn(
            "[IW HUB][LiDAR] Isaac Sim will continue instead of terminating."
        )
    else:
        carb.log_info(
            "[IW HUB][LiDAR] range configured sensors: "
            + ", ".join(configured)
        )

    return configured


class IwHubAgent(BaseRobotAgent):
    """Spawn NVIDIA's IW Hub while limiting only its useful LiDAR range."""

    def __init__(self, cfg, world, usd_path):
        super().__init__(cfg, world)
        self.usd_path = str(usd_path)
        self.spawn_xyz = tuple(cfg["spawn_xyz"])
        self.spawn_yaw = float(cfg.get("spawn_yaw", 0.0))
        self.lidar_min_range_m = float(cfg.get("lidar_min_range_m", 0.8))
        self.lidar_max_range_m = float(cfg.get("lidar_max_range_m", 5.0))
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

        # Preserve NVIDIA's sensor mounting, ROS graph, scan rate, firing rate,
        # tick rate, and horizontal resolution. Only spawn pose and the valid
        # near/far range are overridden locally in this stage.
        transform = UsdGeom.Xformable(prim)
        transform.ClearXformOpOrder()
        transform.AddTranslateOp().Set(Gf.Vec3d(*self.spawn_xyz))
        transform.AddRotateXYZOp().Set(
            Gf.Vec3f(0.0, 0.0, self.spawn_yaw)
        )

        stage.Load(self.prim_path)

        robot_prim = stage.GetPrimAtPath(self.prim_path)
        if not robot_prim.IsValid():
            raise RuntimeError(
                f"IW Hub prim failed to load: {self.prim_path}"
            )

        configured_lidars = _configure_lidar_range(
            robot_prim,
            self.lidar_min_range_m,
            self.lidar_max_range_m,
        )

        if configured_lidars:
            carb.log_info(
                f"[IW HUB] spawned {self.name} at {self.spawn_xyz}; "
                f"LiDAR valid range={self.lidar_min_range_m}-"
                f"{self.lidar_max_range_m} m; "
                "scan rate/resolution/pose=NVIDIA original"
            )
        else:
            carb.log_warn(
                f"[IW HUB] spawned {self.name} at {self.spawn_xyz}; "
                "LiDAR range optimization was skipped."
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
