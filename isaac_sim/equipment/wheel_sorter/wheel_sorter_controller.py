import omni.graph.core as og
import omni.usd


class WheelSorterController:
    """Control the existing wheel-sorter ActionGraph."""

    SORTER_ROOT = "/World/ConveyorTrack/Sorter"
    ACTION_GRAPH_PATH = f"{SORTER_ROOT}/ActionGraph"
    SWITCH_ATTRIBUTE = (
        f"{ACTION_GRAPH_PATH}/binary_switch.inputs:value"
    )

    def __init__(self, toggle_steps: int = 120, speed: float = 1.0):
        self.toggle_steps = int(toggle_steps)
        self.speed = float(speed)
        self.step_count = 0
        self.state = False

    def setup(self):
        """Set a known initial direction and synchronize sorter speed."""

        self.step_count = 0
        self.set_speed(self.speed)
        self.set_state(False)

        print(
            f"[SORTER] binary switch ready: {self.SWITCH_ATTRIBUTE} "
            f"toggle_steps={self.toggle_steps} speed={self.speed:.2f}"
        )

    def _set_graph_variable_speed(self):
        """Set a speed/velocity variable on the sorter graph if it exists."""

        try:
            graph = og.Controller.graph(self.ACTION_GRAPH_PATH)
        except Exception:
            return 0

        context = graph.get_default_graph_context()
        changed = 0

        for name in ("Velocity", "velocity", "Speed", "speed"):
            variable = graph.find_variable(name)
            if variable is None or not variable.valid:
                continue

            if variable.set(context, self.speed):
                changed += 1
                print(
                    f"[SORTER] graph variable {name}={self.speed:.2f}"
                )

        return changed

    def _set_node_speed_inputs(self):
        """Set velocity/speed inputs only inside the sorter subtree."""

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage is not available")

        sorter_prim = stage.GetPrimAtPath(self.SORTER_ROOT)
        if not sorter_prim.IsValid():
            raise RuntimeError(
                f"Wheel sorter prim was not found: {self.SORTER_ROOT}"
            )

        changed = 0

        for prim in stage.Traverse():
            prim_path = str(prim.GetPath())
            if not prim_path.startswith(self.SORTER_ROOT + "/"):
                continue

            # IsaacConveyor-style speed input.
            for input_name in ("inputs:velocity", "inputs:speed"):
                usd_attr = prim.GetAttribute(input_name)
                if not usd_attr:
                    continue

                usd_attr.Set(self.speed)

                og_attr = og.Controller.attribute(
                    f"{prim_path}.{input_name}"
                )
                if og_attr.is_valid():
                    og_attr.set(self.speed)

                changed += 1
                print(
                    f"[SORTER] {prim_path}.{input_name}="
                    f"{self.speed:.2f}"
                )

            # Some graphs feed speed through a ConstantFloat node.
            node_name = prim.GetName().lower()
            if "speed" not in node_name and "velocity" not in node_name:
                continue

            value_attr = prim.GetAttribute("inputs:value")
            if not value_attr:
                continue

            value_attr.Set(self.speed)

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
        """Match the wheel-sorter transport speed to the conveyor speed."""

        self.speed = float(speed)

        changed = 0
        changed += self._set_graph_variable_speed()
        changed += self._set_node_speed_inputs()

        if changed == 0:
            print(
                "[SORTER] WARNING: no speed/velocity input was found "
                "inside the sorter graph"
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
