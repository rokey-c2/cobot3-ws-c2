import omni.graph.core as og


class WheelSorterController:
    """Control the existing wheel-sorter ActionGraph binary switch."""

    SWITCH_ATTRIBUTE = (
        "/World/ConveyorTrack/Sorter/ActionGraph/"
        "binary_switch.inputs:value"
    )

    def __init__(self, toggle_steps: int = 120):
        self.toggle_steps = int(toggle_steps)
        self.step_count = 0
        self.state = False

    def setup(self):
        """Set a known initial sorter direction."""

        self.step_count = 0
        self.set_state(False)
        print(
            f"[SORTER] binary switch ready: {self.SWITCH_ATTRIBUTE} "
            f"toggle_steps={self.toggle_steps}"
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
