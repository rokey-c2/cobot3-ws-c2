import omni.graph.core as og
import omni.usd


CONVEYOR_NODE_TYPE = "isaacsim.asset.gen.conveyor.IsaacConveyor"


class ConveyorController:
    """Control conveyor speed through the existing OmniGraph variables."""

    def __init__(self, speed: float = 1.0):
        self.speed = float(speed)
        self._graph_paths = []
        self.running = False

    def _enable_conveyor_nodes(self, graph_prim):
        """Some plain ConveyorBeltGraph segments in Parcel_Sorting_Map ship
        with their IsaacConveyor node's inputs:enabled left unauthored
        (defaults to disabled) -- Velocity alone does nothing if the node
        itself is off. Force it on for every IsaacConveyor node under this
        graph (found by node:type so it doesn't depend on the node's given
        name)."""

        for prim in graph_prim.GetChildren():
            node_type_attr = prim.GetAttribute("node:type")
            if not node_type_attr or node_type_attr.Get() != CONVEYOR_NODE_TYPE:
                continue
            try:
                # og.Controller.attribute() needs the graph to already be
                # live in the OmniGraph runtime -- before world.reset() the
                # node exists in USD but isn't instantiated yet, so this
                # raises. setup() is called again after reset() (see
                # main_mission.py/main.py), so the retry there succeeds;
                # just skip quietly the first time.
                attribute = og.Controller.attribute(
                    f"{prim.GetPath()}.inputs:enabled"
                )
            except og.OmniGraphError:
                continue
            if attribute.is_valid():
                attribute.set(True)

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

            self._enable_conveyor_nodes(prim)

            # Keep the USD-backed default value in sync as well.
            velocity_attr = prim.GetAttribute("Velocity")
            if not velocity_attr:
                velocity_attr = prim.GetAttribute("velocity")

            if velocity_attr:
                velocity_attr.Set(self.speed)

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
        """Change the runtime speed of every conveyor graph."""

        self.speed = float(speed)
        stage = omni.usd.get_context().get_stage()

        for graph_path in self._graph_paths:
            prim = stage.GetPrimAtPath(graph_path)
            if prim.IsValid():
                velocity_attr = prim.GetAttribute("Velocity")
                if not velocity_attr:
                    velocity_attr = prim.GetAttribute("velocity")

                if velocity_attr:
                    velocity_attr.Set(self.speed)

            self._set_graph_speed(graph_path, self.speed)

    def start(self):
        """Re-apply speed after simulation playback has started."""

        self.set_speed(self.speed)
        self.running = True

    def stop(self):
        """Stop every conveyor graph."""

        for graph_path in self._graph_paths:
            self._set_graph_speed(graph_path, 0.0)
        self.running = False

    def get_status(self):
        return "RUNNING" if self.running else "STOPPED"
