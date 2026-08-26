"""Attach to the IW Hub that is already baked into the map (not spawned)."""

import math

import carb
import numpy as np
from pxr import Sdf, Usd, UsdPhysics
import omni.usd

from robots.base_robot import BaseRobotAgent


_DISCOVERY_TOPICS = (
    "/chassis/odom",
    "/front_2d_lidar/scan",
    "/back_2d_lidar/scan",
)

_LIDAR_RANGE_OFFSET_ATTR = "omni:sensor:Core:rangeOffsetM"
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


def _configure_lidar_range(
    robot_prim,
    range_offset_m,
    min_range_m,
    max_range_m,
):
    """Configure only RTX LiDAR range limits; never move NVIDIA sensors.

    rangeOffsetM makes nearby carried geometry invisible to the ray itself,
    unlike nearRangeM which only rejects a close return after a hit.  This is
    specifically used to prevent the lifted cargo guard from occluding the
    real navigation scene.
    """

    if range_offset_m < 0.0:
        raise ValueError("lidar_range_offset_m must be >= 0")
    if min_range_m <= 0.0:
        raise ValueError("lidar_min_range_m must be > 0")
    if range_offset_m >= min_range_m:
        raise ValueError(
            "lidar_range_offset_m must be smaller than lidar_min_range_m"
        )
    if max_range_m <= min_range_m:
        raise ValueError("lidar_max_range_m must be larger than lidar_min_range_m")

    configured = []
    lidar_like_paths = []

    for lidar_prim in Usd.PrimRange(robot_prim):
        range_offset_attr = lidar_prim.GetAttribute(_LIDAR_RANGE_OFFSET_ATTR)
        near_range_attr = lidar_prim.GetAttribute(_LIDAR_NEAR_RANGE_ATTR)
        far_range_attr = lidar_prim.GetAttribute(_LIDAR_FAR_RANGE_ATTR)
        scan_rate_attr = lidar_prim.GetAttribute(_LIDAR_SCAN_RATE_ATTR)
        firing_rate_attr = lidar_prim.GetAttribute(_LIDAR_FIRING_RATE_ATTR)

        has_range_offset = bool(
            range_offset_attr and range_offset_attr.IsValid()
        )
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
            old_range_offset = (
                range_offset_attr.Get() if has_range_offset else None
            )
        except Exception:
            old_range_offset = None

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

        offset_ok = False
        if has_range_offset:
            offset_ok = _safe_numeric_set(range_offset_attr, range_offset_m)
        else:
            carb.log_warn(
                "[IW HUB][LiDAR] rangeOffsetM attribute not found on "
                f"{lidar_prim.GetPath()}; cargo occlusion protection skipped "
                "for this sensor."
            )

        near_ok = True
        if has_near_range:
            near_ok = _safe_numeric_set(near_range_attr, min_range_m)
        else:
            carb.log_warn(
                "[IW HUB][LiDAR] near-range attribute not found on "
                f"{lidar_prim.GetPath()}; close-return rejection skipped "
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
            f"rangeOffset {old_range_offset} -> "
            f"{range_offset_m if has_range_offset and offset_ok else old_range_offset} m; "
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
    """Locate the IW Hub already baked into the map and tune its LiDAR range.

    Nothing is spawned or moved here -- the robot, its dual LiDAR, and its
    ROS graph are already part of the saved map. This only discovers the
    existing prim (reusing the same odom/LiDAR-topic-based search that used
    to locate the source sub-prim before referencing it) and applies the
    same rangeOffset/near/far LiDAR tuning as before.
    """

    def __init__(self, cfg, world):
        super().__init__(cfg, world)
        self.lidar_range_offset_m = float(
            cfg.get("lidar_range_offset_m", 0.75)
        )
        self.lidar_min_range_m = float(cfg.get("lidar_min_range_m", 0.8))
        self.lidar_max_range_m = float(cfg.get("lidar_max_range_m", 5.0))
        self.prim_path = None

    def setup(self):
        stage = omni.usd.get_context().get_stage()

        self.prim_path = str(_navigation_robot_prim_path(stage))

        robot_prim = stage.GetPrimAtPath(self.prim_path)
        if not robot_prim.IsValid():
            raise RuntimeError(
                f"Baked-in IW Hub prim not found: {self.prim_path}"
            )

        configured_lidars = _configure_lidar_range(
            robot_prim,
            self.lidar_range_offset_m,
            self.lidar_min_range_m,
            self.lidar_max_range_m,
        )

        if configured_lidars:
            carb.log_info(
                f"[IW HUB] attached to baked-in {self.name} at "
                f"{self.prim_path}; "
                f"LiDAR rangeOffset={self.lidar_range_offset_m} m, "
                f"valid range={self.lidar_min_range_m}-"
                f"{self.lidar_max_range_m} m; "
                "scan rate/resolution/pose=map original"
            )
        else:
            carb.log_warn(
                f"[IW HUB] attached to baked-in {self.name} at "
                f"{self.prim_path}; "
                "LiDAR range optimization was skipped."
            )

    def set_map_pose(self, x, y, yaw):
        """Move the existing IW Hub articulation to a verified map pose."""

        values = (float(x), float(y), float(yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("IW Hub restore pose must contain finite values")

        robot = getattr(self, "robot", None)
        if robot is None:
            raise RuntimeError("IW Hub articulation is not ready for restore")

        stop = getattr(self, "_stop", None)
        if callable(stop):
            stop()

        current_position, _ = robot.get_world_pose()
        position = np.array(
            [values[0], values[1], float(current_position[2])],
            dtype=float,
        )
        orientation = np.array(
            [
                math.cos(values[2] / 2.0),
                0.0,
                0.0,
                math.sin(values[2] / 2.0),
            ],
            dtype=float,
        )

        robot.set_linear_velocity(np.zeros(3, dtype=float))
        robot.set_angular_velocity(np.zeros(3, dtype=float))
        robot.set_world_pose(
            position=position,
            orientation=orientation,
        )

        carb.log_info(
            "[IW HUB][RESTORE] map pose applied: "
            f"x={values[0]:.3f}, y={values[1]:.3f}, "
            f"yaw={values[2]:.3f}, z={position[2]:.3f}"
        )

    def post_reset(self):
        stage = omni.usd.get_context().get_stage()
        robot_prim = stage.GetPrimAtPath(self.prim_path)

        if not robot_prim.IsValid():
            raise RuntimeError(
                f"IW Hub prim is missing after reset: {self.prim_path}"
            )

        topic_paths = _topic_node_paths(stage)

        print("\n[IW HUB][DEBUG] ROS2 topic prim paths")

        for topic, paths in topic_paths.items():
            print(f"[IW HUB][DEBUG] {topic}")

            for path in paths:
                if path.HasPrefix(robot_prim.GetPath()):
                    print(f"  {path}")

        print("\n[IW HUB][DEBUG] ROS2 context candidates")

        for prim in Usd.PrimRange(robot_prim):
            path_text = prim.GetPath().pathString.lower()
            type_text = str(prim.GetTypeName()).lower()

            if "context" in path_text or "context" in type_text:
                print(
                    f"  path={prim.GetPath()} "
                    f"type={prim.GetTypeName()}"
                )

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
