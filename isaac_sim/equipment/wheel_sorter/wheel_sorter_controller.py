import math

import omni.graph.core as og
import omni.usd

from pxr import Usd, UsdGeom


class WheelSorterUnit:
    """One existing wheel sorter in ConveyorTrack_01/_02/_03."""

    STRAIGHT_DIRECTION = (1.0, 0.0, 0.0)
    DIVERT_DIRECTION = (1.0, -2.0, 0.0)
    SPEED_TARGET = -1.0

    def __init__(self, track_id: str):
        self.track_id = str(track_id)
        self.track_name = f"ConveyorTrack_{self.track_id}"
        self.sorter_path = f"/World/{self.track_name}/Sorter"
        self.sorter_physics_path = f"{self.sorter_path}/Sorter_physics"
        self.action_graph_path = f"{self.sorter_path}/ActionGraph"
        self.speed_attr_path = (
            f"{self.action_graph_path}/SorterSpeed.inputs:value"
        )
        self.direction_attr_path = (
            f"{self.action_graph_path}/conveyor_belt.inputs:direction"
        )
        self.state = False

    def _get_attribute(self, path: str):
        try:
            attribute = og.Controller.attribute(path)
        except og.OmniGraphError as exc:
            raise RuntimeError(
                f"OmniGraph attribute is not ready: {path}: {exc}"
            ) from exc

        if not attribute.is_valid():
            raise RuntimeError(f"OmniGraph attribute was not found: {path}")
        return attribute

    def setup(self):
        """Apply the fixed speed and default straight direction."""

        self.set_speed(self.SPEED_TARGET)
        self.set_state(False)

    def set_speed(self, speed: float):
        """Set the existing SorterSpeed constant value."""

        attribute = self._get_attribute(self.speed_attr_path)
        attribute.set(float(speed))
        print(
            f"[SORTER] {self.track_name} SorterSpeed={float(speed):.2f}"
        )

    def set_direction(self, direction):
        """Set the existing Isaac Conveyor node's direction input."""

        vector = tuple(float(value) for value in direction)
        if len(vector) != 3:
            raise ValueError("sorter direction must have exactly 3 values")

        attribute = self._get_attribute(self.direction_attr_path)
        attribute.set(vector)
        print(
            f"[SORTER] {self.track_name} direction="
            f"({vector[0]:.1f}, {vector[1]:.1f}, {vector[2]:.1f})"
        )

    def set_state(self, active: bool):
        """Binary-style sorter control without creating an OmniGraph node.

        False / 0 -> (1, 0, 0)
        True  / 1 -> (1, -2, 0)
        """

        self.state = bool(active)
        direction = (
            self.DIVERT_DIRECTION
            if self.state
            else self.STRAIGHT_DIRECTION
        )
        self.set_direction(direction)

    def reset(self):
        self.set_state(False)

    def get_world_xy(self):
        """Read the current world XY position of the existing sorter physics."""

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self.sorter_physics_path)

        if not prim.IsValid():
            prim = stage.GetPrimAtPath(self.sorter_path)

        if not prim.IsValid():
            raise RuntimeError(
                f"sorter prim was not found: {self.sorter_physics_path}"
            )

        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        position = matrix.ExtractTranslation()
        return float(position[0]), float(position[1])


class WheelSorterController:
    """Control the three existing sorters in the current Parcel Sorting Map.

    The old binary_switch/reroute nodes are not used.  Python keeps a simple
    boolean state and maps it to the already-existing conveyor_belt direction
    input.  No ROS2 node or OmniGraph node is created.
    """

    TRACK_IDS = ("01", "02", "03")
    SPEED_TARGET = -1.0
    STRAIGHT_DIRECTION = WheelSorterUnit.STRAIGHT_DIRECTION
    DIVERT_DIRECTION = WheelSorterUnit.DIVERT_DIRECTION

    def __init__(
        self,
        regions=None,
        sorter_speed=None,
        approach_threshold: float = 0.45,
        reset_threshold: float = 0.70,
    ):
        # `regions` and `sorter_speed` remain accepted only so the current
        # main_mission.py does not fail before the demo integration step.
        # The current-map values are fixed by the verified MD specification.
        if regions is not None:
            print(
                "[SORTER] legacy regions argument ignored; "
                "current tracks are 01/02/03"
            )
        if sorter_speed is not None and float(sorter_speed) != self.SPEED_TARGET:
            print(
                f"[SORTER] legacy sorter_speed={sorter_speed} ignored; "
                f"using verified {self.SPEED_TARGET}"
            )

        self.approach_threshold = float(approach_threshold)
        self.reset_threshold = float(reset_threshold)

        if self.reset_threshold <= self.approach_threshold:
            raise ValueError(
                "reset_threshold must be greater than approach_threshold"
            )

        self.units = {
            track_id: WheelSorterUnit(track_id)
            for track_id in self.TRACK_IDS
        }

        # Per-box state used by the demo routing helper.
        self._triggered_pairs = set()
        self._completed_pairs = set()

    def setup(self):
        """Apply verified initial values to all three existing sorters."""

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage is not available")

        for unit in self.units.values():
            sorter_prim = stage.GetPrimAtPath(unit.sorter_path)
            if not sorter_prim.IsValid():
                raise RuntimeError(
                    f"expected sorter was not found: {unit.sorter_path}"
                )
            unit.setup()

        self._triggered_pairs.clear()
        self._completed_pairs.clear()
        print("[SORTER] current-map sorters 01/02/03 ready")

    def start(self):
        """Re-apply fixed speed/default direction after world.play()."""

        for unit in self.units.values():
            unit.setup()

    def set_state(self, track_id, active: bool):
        """Set one sorter using a binary-style False/True interface."""

        key = str(track_id).zfill(2)
        if key not in self.units:
            raise ValueError(
                f"track_id must be one of {self.TRACK_IDS}, got {track_id}"
            )
        self.units[key].set_state(active)

    def set_direction(self, track_id, direction):
        """Direct vector setter for manual verification only."""

        key = str(track_id).zfill(2)
        if key not in self.units:
            raise ValueError(
                f"track_id must be one of {self.TRACK_IDS}, got {track_id}"
            )
        self.units[key].set_direction(direction)

    def reset_sorter(self, track_id):
        self.set_state(track_id, False)

    def reset_all(self):
        for unit in self.units.values():
            unit.reset()

    def verify(self):
        """Print the expected fixed control values for manual validation."""

        for track_id, unit in self.units.items():
            xy = unit.get_world_xy()
            print(
                f"[SORTER][VERIFY] track={track_id} "
                f"world_xy=({xy[0]:.3f}, {xy[1]:.3f}) "
                f"speed={self.SPEED_TARGET:.1f} state={int(unit.state)}"
            )

    def _box_world_xy(self, box_path):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(box_path)
        if not prim.IsValid():
            return None

        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        position = matrix.ExtractTranslation()
        return float(position[0]), float(position[1])

    def _read_box_id(self, box_path):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(box_path)
        if not prim.IsValid():
            return None

        attribute = prim.GetAttribute("box_id")
        if not attribute or not attribute.IsValid():
            return None

        value = attribute.Get()
        return None if value is None else int(value)

    def update_boxes(self, box_prim_paths, box_id_to_track):
        """Route demo boxes by proximity to the three physical sorters.

        `box_id_to_track` is deliberately supplied by the caller.  The code
        does NOT invent the user's final routing policy.  Example shape only:

            {10: "01", 20: "02", 30: "03"}

        When a box enters a sorter's approach radius, that sorter becomes
        DIVERT only if this box's configured target is that track; otherwise
        it stays STRAIGHT.  After the box leaves the reset radius, the sorter
        returns to STRAIGHT automatically.
        """

        for box_path in tuple(box_prim_paths):
            box_xy = self._box_world_xy(box_path)
            if box_xy is None:
                continue

            box_id = self._read_box_id(box_path)
            if box_id is None:
                continue

            target_track = box_id_to_track.get(box_id)
            if target_track is not None:
                target_track = str(target_track).zfill(2)
                if target_track not in self.units:
                    raise ValueError(
                        f"box_id_to_track[{box_id}] points to invalid "
                        f"track {target_track}"
                    )

            for track_id, unit in self.units.items():
                pair = (box_path, track_id)
                if pair in self._completed_pairs:
                    continue

                sorter_xy = unit.get_world_xy()
                distance = math.hypot(
                    box_xy[0] - sorter_xy[0],
                    box_xy[1] - sorter_xy[1],
                )

                if pair not in self._triggered_pairs:
                    if distance > self.approach_threshold:
                        continue

                    should_divert = target_track == track_id
                    unit.set_state(should_divert)
                    self._triggered_pairs.add(pair)
                    print(
                        f"[SORTER] box={box_path} box_id={box_id} "
                        f"reached track={track_id} distance={distance:.3f} "
                        f"state={int(should_divert)}"
                    )
                    continue

                if distance >= self.reset_threshold:
                    unit.reset()
                    self._completed_pairs.add(pair)
                    print(
                        f"[SORTER] box={box_path} passed track={track_id}; "
                        "direction reset to (1, 0, 0)"
                    )

    def route_box(self, destination: str):
        """Compatibility hook for the existing single-box mission.

        Existing main_mission/P3020 code currently calls route_box("A"/"B"/
        "C").  Keep that call alive until demo 1 is merged, but implement it
        with the new direction-vector controller rather than binary_switch.
        """

        compatibility_map = {
            "A": "01",
            "B": "02",
            "C": "03",
        }
        target_track = compatibility_map.get(str(destination).upper())

        for track_id, unit in self.units.items():
            unit.set_state(track_id == target_track)

        if target_track is None:
            print(
                f"[SORTER] compatibility destination={destination!r}: "
                "all sorters STRAIGHT"
            )
        else:
            print(
                f"[SORTER] compatibility destination={destination!r} "
                f"-> ConveyorTrack_{target_track}"
            )

        return destination
