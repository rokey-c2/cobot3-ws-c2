import re

import omni.graph.core as og
import omni.usd


class WheelSorterController:
    """Control the existing wheel-sorter ActionGraph."""

    SORTER_ROOT = "/World/ConveyorTrack/Sorter"
    ACTION_GRAPH_PATH = f"{SORTER_ROOT}/ActionGraph"
    SWITCH_ATTRIBUTE = (
        f"{ACTION_GRAPH_PATH}/binary_switch.inputs:value"
    )

    def __init__(self, toggle_steps: int = 120, speed: float = -1.0):
        self.toggle_steps = int(toggle_steps)
        self.speed = float(speed)
        self.step_count = 0
        self.state = False

    def setup(self):
        """Set a known initial direction and apply the sorter speed."""

        self.step_count = 0
        self.set_speed(self.speed)
        self.set_state(False)

        print(
            f"[SORTER] binary switch ready: {self.SWITCH_ATTRIBUTE} "
            f"toggle_steps={self.toggle_steps} speed={self.speed:.2f}"
        )

    def _set_graph_variable_speed(self):
        """Set the ActionGraph variable used by the wheel sorter speed."""

        try:
            graph = og.Controller.graph(self.ACTION_GRAPH_PATH)
        except Exception:
            return 0

        context = graph.get_default_graph_context()
        changed = 0

        variable_names = (
            "Sorter Speed",
            "SorterSpeed",
            "sorter_speed",
            "sorterSpeed",
            "Velocity",
            "velocity",
            "Speed",
            "speed",
        )

        for name in variable_names:
            variable = graph.find_variable(name)
            if variable is None or not variable.valid:
                continue

            if variable.set(context, self.speed):
                changed += 1
                print(
                    f"[SORTER] graph variable {name}={self.speed:.2f}"
                )

        return changed

    @staticmethod
    def _normalize_name(name: str):
        return re.sub(r"[^a-z0-9]", "", name.lower())

    def _set_sorter_speed_usd_attributes(self):
        """Find the USD attribute shown as 'Sorter Speed' and set it."""

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage is not available")

        action_graph = stage.GetPrimAtPath(self.ACTION_GRAPH_PATH)
        if not action_graph.IsValid():
            raise RuntimeError(
                f"Wheel sorter ActionGraph was not found: {self.ACTION_GRAPH_PATH}"
            )

        changed = 0

        for prim in stage.Traverse():
            prim_path = str(prim.GetPath())
            if not (
                prim_path == self.ACTION_GRAPH_PATH
                or prim_path.startswith(self.ACTION_GRAPH_PATH + "/")
            ):
                continue

            for attr in prim.GetAttributes():
                attr_name = attr.GetName()
                normalized = self._normalize_name(attr_name)

                # Match the actual property displayed in the graph as
                # "Sorter Speed" without depending on USD punctuation.
                if "sorter" not in normalized or "speed" not in normalized:
                    continue

                try:
                    attr.Set(self.speed)
                except Exception:
                    continue

                changed += 1
                print(
                    f"[SORTER] {prim_path}.{attr_name}={self.speed:.2f}"
                )

            node_name = self._normalize_name(prim.GetName())
            if "sorter" not in node_name or "speed" not in node_name:
                continue

            value_attr = prim.GetAttribute("inputs:value")
            if not value_attr:
                continue

            try:
                value_attr.Set(self.speed)
            except Exception:
                continue

            og_attr = og.Controller.attribute(
                f"{prim_path}.inputs:value"
            )
            if og_attr.is_valid():
                og_attr.set(self.speed)

            changed += 1
            print(
                f"[SORTER] {prim_path}.inputs:value={self.speed:.2f}"
            )

        return changed

    def set_speed(self, speed: float):
        """Set the signed wheel-sorter speed.

        The existing graph uses a negative value for the current forward
        direction, so -1.0 matches the conveyor's 1.0 speed magnitude.
        """

        self.speed = float(speed)

        changed = 0
        changed += self._set_graph_variable_speed()
        changed += self._set_sorter_speed_usd_attributes()

        if changed == 0:
            print(
                "[SORTER] WARNING: 'Sorter Speed' property was not found "
                "inside the ActionGraph"
            )

    def set_state(self, state: bool):
        """Set the ActionGraph binary switch directly."""

        self.state = bool(state)
        attribute = og.Controller.attribute(self.SWITCH_ATTRIBUTE)

        if not attribute.is_valid():
            raise RuntimeError(
                "Wheel sorter binary switch was not found: "
                f"{self.SWITCH_ATTRIBUTE}"
            )

        attribute.set(self.state)

    def update(self):
        """Temporary test mode: toggle direction every N simulation steps."""

        self.step_count += 1

        if self.step_count < self.toggle_steps:
            return

        self.step_count = 0
        self.set_state(not self.state)
        print(f"[SORTER] binary switch={self.state}")

    def route(self, destination: str):
        """Manual A/B routing hook for the future sorting condition."""

        if destination not in ("A", "B"):
            raise ValueError("destination must be 'A' or 'B'")

        self.set_state(destination == "B")
        return destination
