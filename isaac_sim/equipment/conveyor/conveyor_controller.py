import omni.usd


class ConveyorController:
    """Control the conveyor belt graph velocities in the loaded warehouse."""

    def __init__(self, speed: float = 1.0):
        self.speed = float(speed)
        self._graph_paths = []

    def setup(self):
        """Find conveyor graphs and set their initial speed."""

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage is not available")

        self._graph_paths.clear()

        for prim in stage.Traverse():
            if not prim.GetName().startswith("ConveyorBeltGraph"):
                continue

            velocity_attr = prim.GetAttribute("Velocity")
            if not velocity_attr:
                velocity_attr = prim.GetAttribute("velocity")

            if not velocity_attr:
                continue

            velocity_attr.Set(self.speed)
            graph_path = str(prim.GetPath())
            self._graph_paths.append(graph_path)
            print(f"[CONVEYOR] {graph_path} speed={self.speed:.2f}")

        if not self._graph_paths:
            print("[CONVEYOR] WARNING: no ConveyorBeltGraph velocity found")

    def set_speed(self, speed: float):
        """Change the speed of every conveyor graph found during setup."""

        self.speed = float(speed)
        stage = omni.usd.get_context().get_stage()

        for graph_path in self._graph_paths:
            prim = stage.GetPrimAtPath(graph_path)
            if not prim.IsValid():
                continue

            velocity_attr = prim.GetAttribute("Velocity")
            if not velocity_attr:
                velocity_attr = prim.GetAttribute("velocity")

            if velocity_attr:
                velocity_attr.Set(self.speed)

    def start(self):
        self.set_speed(self.speed)

    def stop(self):
        self.set_speed(0.0)
