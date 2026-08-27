import omni.graph.core as og
import omni.usd

from isaacsim.core.utils.extensions import enable_extension


CONVEYOR_NODE_TYPE = "isaacsim.asset.gen.conveyor.IsaacConveyor"
CONVEYOR_EXTENSION = "isaacsim.asset.gen.conveyor"


class ConveyorController:
    """Control the existing conveyor OmniGraphs in the current parcel map.

    No OmniGraph nodes are created here -- this only changes values on
    graphs that already exist in the USD stage. Per-graph speeds come from
    hwi_new_sorter's verified Demo2 run: a single speed applied to every
    ConveyorBeltGraph is wrong, because ConveyorTrack_01/02/03's second
    graph must run at -1.0 (not +1.0), and ConveyorTrack_05 must not be
    touched at all (role not confirmed yet).

    Mission rule: once the simulation is running, an owned conveyor graph
    must be physically active. setup()/start() therefore explicitly enable
    Isaac's conveyor runtime extension and re-enable every IsaacConveyor
    node before applying velocity.
    """

    SORTER_TRACKS = ("01", "02", "03")
    SINGLE_GRAPH_TRACKS = ("04", "06", "07", "10", "11", "16", "17")
    EXCLUDED_TRACKS = ("05",)

    MAIN_GRAPH_SPEED = 1.0
    SORTER_SECOND_GRAPH_SPEED = -1.0

    def __init__(self, speed=None):
        if speed is not None and float(speed) != self.MAIN_GRAPH_SPEED:
            print(
                f"[CONVEYOR] legacy speed={speed} ignored; using the "
                "verified per-track graph speed policy"
            )
        self._graph_speeds = {}
        self.running = False

    @staticmethod
    def _ensure_conveyor_extension():
        """Make sure IsaacConveyor node types are available at runtime."""

        enable_extension(CONVEYOR_EXTENSION)

    def _expected_graph_speeds(self):
        """Return the exact graph paths and their target velocities."""

        graph_speeds = {
            "/World/ConveyorTrack/ConveyorBeltGraph": self.MAIN_GRAPH_SPEED,
        }

        for track_id in self.SORTER_TRACKS:
            track_path = f"/World/ConveyorTrack_{track_id}"
            graph_speeds[f"{track_path}/ConveyorBeltGraph"] = self.MAIN_GRAPH_SPEED
            graph_speeds[f"{track_path}/ConveyorBeltGraph_01"] = self.SORTER_SECOND_GRAPH_SPEED

        for track_id in self.SINGLE_GRAPH_TRACKS:
            track_path = f"/World/ConveyorTrack_{track_id}"
            graph_speeds[f"{track_path}/ConveyorBeltGraph"] = self.MAIN_GRAPH_SPEED

        return graph_speeds

    def _enable_conveyor_nodes(self, graph_prim):
        """Force every IsaacConveyor node under one graph to enabled=True."""

        for prim in graph_prim.GetChildren():
            node_type_attr = prim.GetAttribute("node:type")
            if not node_type_attr or node_type_attr.Get() != CONVEYOR_NODE_TYPE:
                continue

            try:
                attribute = og.Controller.attribute(
                    f"{prim.GetPath()}.inputs:enabled"
                )
            except og.OmniGraphError:
                continue

            if attribute.is_valid():
                attribute.set(True)

    def _find_velocity_variable(self, graph):
        for name in ("Velocity", "velocity"):
            variable = graph.find_variable(name)
            if variable is not None and variable.valid:
                return variable
        return None

    def _set_usd_velocity(self, graph_path: str, speed: float):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(graph_path)
        if not prim.IsValid():
            return False

        velocity_attr = prim.GetAttribute("Velocity")
        if not velocity_attr:
            velocity_attr = prim.GetAttribute("velocity")

        if velocity_attr:
            velocity_attr.Set(float(speed))

        return True

    def _set_runtime_velocity(self, graph_path: str, speed: float):
        try:
            graph = og.Controller.graph(graph_path)
        except Exception as exc:
            print(f"[CONVEYOR] runtime graph not ready: {graph_path}: {exc}")
            return False

        variable = self._find_velocity_variable(graph)
        if variable is None:
            print(f"[CONVEYOR] WARNING: Velocity variable not found: {graph_path}")
            return False

        context = graph.get_default_graph_context()
        if not variable.set(context, float(speed)):
            print(f"[CONVEYOR] ERROR: runtime Velocity set failed: {graph_path}")
            return False

        runtime_speed = variable.get(context)
        print(f"[CONVEYOR] {graph_path} Velocity={float(runtime_speed):.2f}")
        return True

    def setup(self):
        """Apply all authored initial conveyor values for the current map."""

        self._ensure_conveyor_extension()

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage is not available")

        self._graph_speeds = self._expected_graph_speeds()

        for graph_path, speed in self._graph_speeds.items():
            prim = stage.GetPrimAtPath(graph_path)
            if not prim.IsValid():
                print(f"[CONVEYOR] WARNING: expected graph missing: {graph_path}")
                continue

            self._enable_conveyor_nodes(prim)
            self._set_usd_velocity(graph_path, speed)
            self._set_runtime_velocity(graph_path, speed)

        print("[CONVEYOR] configured current map; ConveyorTrack_05 intentionally untouched")

    def start(self):
        """Physically enable owned conveyor nodes and apply running speeds."""

        self._ensure_conveyor_extension()
        stage = omni.usd.get_context().get_stage()

        for graph_path, speed in self._graph_speeds.items():
            prim = stage.GetPrimAtPath(graph_path)
            if not prim.IsValid():
                print(f"[CONVEYOR] WARNING: expected graph missing: {graph_path}")
                continue

            # Important: Velocity alone does nothing when IsaacConveyor's
            # inputs:enabled is false. Re-enable it every time START is
            # requested so a box placed on the belt immediately flows.
            self._enable_conveyor_nodes(prim)
            self._set_usd_velocity(graph_path, speed)
            self._set_runtime_velocity(graph_path, speed)

        self.running = True
        print("[CONVEYOR] START confirmed: conveyor nodes enabled and speeds applied")

    def stop(self):
        """Stop only the graphs owned by this controller."""

        for graph_path in self._graph_speeds:
            self._set_runtime_velocity(graph_path, 0.0)
        self.running = False

    def get_status(self):
        return "RUNNING" if self.running else "STOPPED"

    def verify(self):
        """Print current runtime values without changing the configured map."""

        for graph_path, expected in self._graph_speeds.items():
            try:
                graph = og.Controller.graph(graph_path)
            except Exception as exc:
                print(f"[CONVEYOR][VERIFY] graph unavailable: {graph_path}: {exc}")
                continue

            variable = self._find_velocity_variable(graph)
            if variable is None:
                print(f"[CONVEYOR][VERIFY] Velocity missing: {graph_path}")
                continue

            context = graph.get_default_graph_context()
            value = float(variable.get(context))
            print(f"[CONVEYOR][VERIFY] {graph_path} current={value:.2f} expected={expected:.2f}")