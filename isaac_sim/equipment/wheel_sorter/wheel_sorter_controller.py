import math

import omni.graph.core as og
import omni.usd

from pxr import Usd, UsdGeom


class WheelSorterUnit:
    """One physical wheel-sorter (a ConveyorTrack_XX/Sorter/ActionGraph).

    Direction (1,0,0) lets a box continue straight down the main line;
    (1,-2,0) diverts it off the line into this unit's region. These paths
    and values come from hwi_new_sorter's verified Demo2 run -- the earlier
    reroute/binary_switch node search was for a different, never-finished
    sorter graph design.
    """

    STRAIGHT_DIRECTION = (1.0, 0.0, 0.0)
    DIVERT_DIRECTION = (1.0, -2.0, 0.0)

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

    def setup(self, speed: float):
        # main_mission.py calls setup() twice: once before world.reset()/
        # play() (when this ActionGraph exists in USD but isn't live in the
        # OmniGraph runtime yet -- same race ConveyorController's
        # _enable_conveyor_nodes already works around) and once after, when
        # it's actually ready. Skip quietly on the first pass instead of
        # crashing the whole Isaac Sim process; the post-reset call succeeds
        # for real.
        try:
            self.set_speed(speed)
            self.set_state(False)
        except RuntimeError as exc:
            print(
                f"[SORTER] {self.track_name} not live yet "
                f"(expected before world.reset()): {exc}"
            )

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
    """Physical box_id-based routing (verified on hwi_new_sorter's Demo2),
    plus the control-tower region START/STOP + process-event interface that
    main_mission.py's ProcessEquipmentBridge already depends on.

    Physical routing truth is box_id, read directly off each parcel prim's
    "box_id" attribute -- NOT the "destination" attribute. destination stays
    on the prim for dashboard/business display only (see
    docs/sorter_merge/SORTER_MERGE_PLAN.md section 7). Region letters
    (A/B/C, used by the dashboard's per-sorter START/STOP controls) map
    positionally onto the verified physical tracks (01/02/03).
    """

    TRACK_IDS = ("01", "02", "03")
    SPEED_TARGET = -1.0
    REGION_TO_TRACK = {"A": "01", "B": "02", "C": "03"}

    def __init__(
        self,
        regions=("A", "B", "C"),
        sorter_speed=None,
        approach_threshold: float = 0.45,
        reset_threshold: float = 0.70,
    ):
        if tuple(regions) != ("A", "B", "C"):
            print(
                f"[SORTER] regions={regions} ignored; region letters are "
                "fixed to A/B/C mapped onto verified tracks 01/02/03"
            )
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
        self.enabled = {track_id: True for track_id in self.TRACK_IDS}
        self._triggered_pairs = set()
        self._completed_pairs = set()
        self._process_events = []

    def setup(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage is not available")

        for track_id, unit in self.units.items():
            sorter_prim = stage.GetPrimAtPath(unit.sorter_path)
            if not sorter_prim.IsValid():
                raise RuntimeError(f"expected sorter was not found: {unit.sorter_path}")
            unit.setup(self.SPEED_TARGET if self.enabled[track_id] else 0.0)

        self._triggered_pairs.clear()
        self._completed_pairs.clear()
        print(
            "[SORTER] current-map sorters 01/02/03 ready; "
            f"approach={self.approach_threshold:.2f} m; "
            f"reset={self.reset_threshold:.2f} m"
        )

    def start(self):
        for track_id, unit in self.units.items():
            unit.setup(self.SPEED_TARGET if self.enabled[track_id] else 0.0)

    def verify(self):
        for track_id, unit in self.units.items():
            xy = unit.get_world_xy()
            print(
                f"[SORTER][VERIFY] track={track_id} "
                f"world_xy=({xy[0]:.3f}, {xy[1]:.3f}) "
                f"speed={self.SPEED_TARGET:.1f} state={int(unit.state)}"
            )

    def set_region_enabled(self, region: str, enabled: bool):
        track_id = self.REGION_TO_TRACK.get(str(region).strip().upper())
        unit = self.units.get(track_id)
        if unit is None:
            return False
        enabled = bool(enabled)
        self.enabled[track_id] = enabled
        unit.reset()
        unit.set_speed(self.SPEED_TARGET if enabled else 0.0)
        return True

    def get_region_status(self, region: str):
        track_id = self.REGION_TO_TRACK.get(str(region).strip().upper())
        if track_id is None or not self.enabled.get(track_id, False):
            return "STOPPED"
        return "RUNNING"

    def take_process_events(self):
        events = list(self._process_events)
        self._process_events.clear()
        return events

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
        """Call every simulation tick (including from inside P3020's
        blocking pick/place loop via tick_others) with the live list of
        parcel prim paths still on the map and a {box_id: track_id} routing
        table. A box_id absent from the table, or mapped to None, passes
        every sorter straight through."""

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

                    should_divert = self.enabled.get(track_id, True) and target_track == track_id
                    unit.set_state(should_divert)
                    self._triggered_pairs.add(pair)
                    print(
                        f"[SORTER] box={box_path} box_id={box_id} "
                        f"reached track={track_id} distance={distance:.3f} "
                        f"state={int(should_divert)}"
                    )
                    if should_divert:
                        region = next(
                            (r for r, t in self.REGION_TO_TRACK.items() if t == track_id),
                            track_id,
                        )
                        self._process_events.append({"state": f"ROUTING:{region}"})
                    continue

                if distance >= self.reset_threshold:
                    unit.reset()
                    self._completed_pairs.add(pair)
                    print(
                        f"[SORTER] box={box_path} passed track={track_id}; "
                        "direction reset to (1, 0, 0)"
                    )
                    if target_track == track_id:
                        region = next(
                            (r for r, t in self.REGION_TO_TRACK.items() if t == track_id),
                            track_id,
                        )
                        self._process_events.append({"state": f"ARRIVED:{region}"})

    def route_box(self, destination: str):
        """Compatibility shim for old destination-based callers -- no
        longer the physical routing path (see update_boxes()). Kept only
        in case something outside this module still calls it."""

        track_id = self.REGION_TO_TRACK.get(str(destination).strip().upper())
        for unit_track_id, unit in self.units.items():
            unit.set_state(unit_track_id == track_id)
        return destination
