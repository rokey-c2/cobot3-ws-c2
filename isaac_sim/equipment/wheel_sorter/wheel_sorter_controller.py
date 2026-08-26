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
        self.speed_attr_path = f"{self.action_graph_path}/SorterSpeed.inputs:value"
        self.direction_attr_path = f"{self.action_graph_path}/conveyor_belt.inputs:direction"
        self.state = False

    def _get_attribute(self, path: str):
        try:
            attribute = og.Controller.attribute(path)
        except og.OmniGraphError as exc:
            raise RuntimeError(f"OmniGraph attribute is not ready: {path}: {exc}") from exc
        if not attribute.is_valid():
            raise RuntimeError(f"OmniGraph attribute was not found: {path}")
        return attribute

    def setup(self):
        self.set_speed(self.SPEED_TARGET)
        self.set_state(False)

    def set_speed(self, speed: float):
        attribute = self._get_attribute(self.speed_attr_path)
        attribute.set(float(speed))
        print(f"[SORTER] {self.track_name} SorterSpeed={float(speed):.2f}")

    def set_direction(self, direction):
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
        self.state = bool(active)
        direction = self.DIVERT_DIRECTION if self.state else self.STRAIGHT_DIRECTION
        self.set_direction(direction)

    def reset(self):
        self.set_state(False)

    def get_world_xy(self):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self.sorter_physics_path)
        if not prim.IsValid():
            prim = stage.GetPrimAtPath(self.sorter_path)
        if not prim.IsValid():
            raise RuntimeError(f"sorter prim was not found: {self.sorter_physics_path}")
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position = matrix.ExtractTranslation()
        return float(position[0]), float(position[1])


class WheelSorterController:
    """Control the three existing sorters in the current Parcel Sorting Map."""

    TRACK_IDS = ("01", "02", "03")
    SPEED_TARGET = -1.0

    def __init__(
        self,
        regions=None,
        sorter_speed=None,
        approach_threshold: float = 0.45,
        reset_threshold: float = 0.70,
    ):
        if regions is not None:
            print("[SORTER] legacy regions argument ignored; current tracks are 01/02/03")
        if sorter_speed is not None and float(sorter_speed) != self.SPEED_TARGET:
            print(
                f"[SORTER] legacy sorter_speed={sorter_speed} ignored; "
                f"using verified {self.SPEED_TARGET}"
            )

        self.approach_threshold = float(approach_threshold)
        self.reset_threshold = float(reset_threshold)
        if self.approach_threshold <= 0.0:
            raise ValueError("approach_threshold must be greater than 0")
        if self.reset_threshold <= self.approach_threshold:
            raise ValueError("reset_threshold must be greater than approach_threshold")

        self.units = {track_id: WheelSorterUnit(track_id) for track_id in self.TRACK_IDS}
        self._triggered_pairs = set()
        self._completed_pairs = set()

    def setup(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage is not available")

        for unit in self.units.values():
            sorter_prim = stage.GetPrimAtPath(unit.sorter_path)
            if not sorter_prim.IsValid():
                raise RuntimeError(f"expected sorter was not found: {unit.sorter_path}")
            unit.setup()

        self._triggered_pairs.clear()
        self._completed_pairs.clear()
        print(
            "[SORTER] current-map sorters 01/02/03 ready; "
            f"approach={self.approach_threshold:.2f} m; "
            f"reset={self.reset_threshold:.2f} m"
        )

    def start(self):
        for unit in self.units.values():
            unit.setup()

    def set_state(self, track_id, active: bool):
        key = str(track_id).zfill(2)
        if key not in self.units:
            raise ValueError(f"track_id must be one of {self.TRACK_IDS}, got {track_id}")
        self.units[key].set_state(active)

    def reset_all(self):
        for unit in self.units.values():
            unit.reset()

    def verify(self):
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
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
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
                        f"box_id_to_track[{box_id}] points to invalid track {target_track}"
                    )

            for track_id, unit in self.units.items():
                pair = (box_path, track_id)
                if pair in self._completed_pairs:
                    continue

                sorter_xy = unit.get_world_xy()
                distance = math.hypot(box_xy[0] - sorter_xy[0], box_xy[1] - sorter_xy[1])

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
        compatibility_map = {"A": "01", "B": "02", "C": "03"}
        target_track = compatibility_map.get(str(destination).upper())
        for track_id, unit in self.units.items():
            unit.set_state(track_id == target_track)
        return destination
