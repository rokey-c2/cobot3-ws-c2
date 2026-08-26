import omni.graph.core as og
import omni.usd

from pxr import Usd, UsdGeom


class WheelSorterController:
    """Reference copy from hwi_conveyor_test. Do not use as the current controller."""

    SWITCH_ATTRIBUTE = (
        "/World/ConveyorTrack/Sorter/ActionGraph/"
        "binary_switch.inputs:value"
    )
    SPEED_ATTRIBUTE = (
        "/World/ConveyorTrack/Sorter/ActionGraph/"
        "SorterSpeed.inputs:value"
    )
    ARRIVAL_TRIGGER_X = -1.0
    SPEED_TARGET = -1.0

    def __init__(self, toggle_steps: int = 120):
        self.toggle_steps = int(toggle_steps)
        self.step_count = 0
        self.state = False
        self._routed_box_paths = set()

    def setup(self):
        self.step_count = 0
        self.set_state(False)
        self._apply_speed_target()

    def _apply_speed_target(self):
        attribute = og.Controller.attribute(self.SPEED_ATTRIBUTE)
        if not attribute.is_valid():
            raise RuntimeError(
                "Wheel sorter speed attribute was not found: "
                f"{self.SPEED_ATTRIBUTE}"
            )
        attribute.set(self.SPEED_TARGET)

    def set_state(self, state: bool):
        self.state = bool(state)
        attribute = og.Controller.attribute(self.SWITCH_ATTRIBUTE)
        if not attribute.is_valid():
            raise RuntimeError(
                "Wheel sorter binary switch was not found: "
                f"{self.SWITCH_ATTRIBUTE}"
            )
        attribute.set(self.state)

    def update(self):
        self.step_count += 1
        if self.step_count < self.toggle_steps:
            return
        self.step_count = 0
        self.set_state(not self.state)

    def route(self, destination: str):
        if destination not in ("A", "B"):
            raise ValueError("destination must be 'A' or 'B'")
        self.set_state(destination == "B")
        return destination

    def route_boxes(self, box_prim_paths):
        stage = omni.usd.get_context().get_stage()

        for box_path in box_prim_paths:
            if box_path in self._routed_box_paths:
                continue

            prim = stage.GetPrimAtPath(box_path)
            if not prim.IsValid():
                continue

            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
            box_position = matrix.ExtractTranslation()

            if box_position[0] > self.ARRIVAL_TRIGGER_X:
                continue

            box_id_attr = prim.GetAttribute("box_id")
            if not box_id_attr.IsValid():
                self._routed_box_paths.add(box_path)
                continue

            box_id = int(box_id_attr.Get())
            self.set_state(bool(box_id))
            self._routed_box_paths.add(box_path)
