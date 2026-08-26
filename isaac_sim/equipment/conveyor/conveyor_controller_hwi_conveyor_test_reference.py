import omni.graph.core as og
import omni.usd


class ConveyorController:
    """Control conveyor speed through the existing OmniGraph variables."""

    # Every belt runs at +speed EXCEPT these exact prim paths, which run at
    # -speed. Confirmed against the actual runtime log: only this one belt
    # needs to run backward, every other belt (including ConveyorTrack_01/
    # _02/_03 and ConveyorBeltGraph_01) is correct at +speed.
    REVERSED_GRAPH_PATHS = frozenset({
        "/World/ConveyorTrack/ConveyorBeltGraph",
    })

    def __init__(self, speed: float = 1.0):
        self.speed = float(speed)
        self._graph_paths = []

    def _speed_for(self, graph_path: str) -> float:
        """Return the signed speed to apply to a specific belt graph."""

        if graph_path in self.REVERSED_GRAPH_PATHS:
            return -self.speed

        return self.speed

    def setup(self):
        """Find ConveyorBeltGraph prims and apply the initial speed."""

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage is not available")

        self._graph_paths.clear()

        for prim in stage.Traverse():
            if not prim.GetName().startswith("ConveyorBeltGraph"):
                continue

            graph_path = str(prim.GetPath())
            self._graph_paths.append(graph_path)

            velocity_attr = prim.GetAttribute("Velocity")
            if not velocity_attr:
                velocity_attr = prim.GetAttribute("velocity")

            if velocity_attr:
                velocity_attr.Set(self._speed_for(graph_path))

        if not self._graph_paths:
            print("[CONVEYOR] WARNING: no ConveyorBeltGraph found")
            return

        self.set_speed(self.speed)

    def _find_velocity_variable(self, graph):
        """Return the graph's Velocity variable."""

        for name in ("Velocity", "velocity"):
            variable = graph.find_variable(name)
            if variable is not None and variable.valid:
                return variable

        return None

    def _set_graph_speed(self, graph_path: str, speed: float):
        """Set the current OmniGraph runtime velocity."""

        try:
            graph = og.Controller.graph(graph_path)
        except Exception as exc:
            print(
                f"[CONVEYOR] ERROR: graph lookup failed "
                f"{graph_path}: {exc}"
            )
            return

        variable = self._find_velocity_variable(graph)

        if variable is None:
            print(
                f"[CONVEYOR] WARNING: Velocity variable not found: "
                f"{graph_path}"
            )
            return

        context = graph.get_default_graph_context()

        if not variable.set(context, float(speed)):
            print(
                f"[CONVEYOR] ERROR: runtime Velocity set failed: "
                f"{graph_path}"
            )
            return

        runtime_speed = variable.get(context)
        print(
            f"[CONVEYOR] graph={graph_path} "
            f"runtime_speed={float(runtime_speed):.2f}"
        )

    def set_speed(self, speed: float):
        """Change the runtime speed of every conveyor graph.

        `speed` is the sign/magnitude applied to every belt except the
        ones in `REVERSED_GRAPH_PATHS`, which run at `-speed`.
        """

        self.speed = float(speed)
        stage = omni.usd.get_context().get_stage()

        for graph_path in self._graph_paths:
            target_speed = self._speed_for(graph_path)
            prim = stage.GetPrimAtPath(graph_path)
            if prim.IsValid():
                velocity_attr = prim.GetAttribute("Velocity")
                if not velocity_attr:
                    velocity_attr = prim.GetAttribute("velocity")

                if velocity_attr:
                    velocity_attr.Set(target_speed)

            self._set_graph_speed(graph_path, target_speed)

    def start(self):
        """Re-apply speed after simulation playback has started."""

        self.set_speed(self.speed)

    def stop(self):
        """Stop every conveyor graph."""

        for graph_path in self._graph_paths:
            self._set_graph_speed(graph_path, 0.0)
