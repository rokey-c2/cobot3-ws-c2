import omni.graph.core as og
import omni.usd


# 분기 스위치 노드 이름 후보. 지금 맵(WIP)은 "reroute"를 쓰지만, 표준 Isaac
# Sim conveyor-split 예제는 "binary_switch"를 쓴다 -- 사용자가 소터/컨베이어를
# 직접 다시 만들기로 했으니, 어느 쪽으로 만들어도 자동으로 찾도록 둘 다 시도한다.
SWITCH_NODE_CANDIDATES = ("reroute", "binary_switch")
# 소터 자체 벨트 속도 노드 이름 후보 (지금 맵의 "SorterSpeed", 또는 표준
# 예제의 컨베이어 그래프 변수 "Velocity"에 대응하는 상수 노드).
SPEED_NODE_CANDIDATES = ("SorterSpeed", "Velocity")


def _resolve_attr_path(action_graph_path: str, node_candidates):
    """action_graph_path 밑에서 node_candidates 중 실제로 존재하는 노드의
    inputs:value 속성 경로를 찾아서 돌려준다. 없으면 None."""
    for node_name in node_candidates:
        path = f"{action_graph_path}/{node_name}.inputs:value"
        if og.Controller.attribute(path).is_valid():
            return path
    return None


class WheelSorterUnit:
    """One physical wheel-sorter (a ConveyorTrack_XX/Sorter/ActionGraph).

    Diverting the switch (reroute/binary_switch) True routes a box off the
    main line into this unit's region; False lets it continue straight down
    the line to the next sorter (or, past the last one, to the end-of-line
    배송지 오류 section).
    """

    def __init__(self, track_name: str, action_graph_path: str):
        self.track_name = track_name
        self.action_graph_path = action_graph_path
        self.switch_attr_path = _resolve_attr_path(
            action_graph_path, SWITCH_NODE_CANDIDATES
        )
        self.speed_attr_path = _resolve_attr_path(
            action_graph_path, SPEED_NODE_CANDIDATES
        )
        if self.switch_attr_path is None:
            raise RuntimeError(
                f"no divert-switch node ({SWITCH_NODE_CANDIDATES}) found "
                f"under {action_graph_path}"
            )

    def set_reroute(self, active: bool):
        og.Controller.attribute(self.switch_attr_path).set(bool(active))

    def set_speed(self, speed: float):
        if self.speed_attr_path is None:
            print(
                f"[SORTER][WARN] {self.track_name}: no speed node "
                f"({SPEED_NODE_CANDIDATES}) found, skipping"
            )
            return
        og.Controller.attribute(self.speed_attr_path).set(float(speed))


class WheelSorterController:
    """Discovers every ConveyorTrack_XX/Sorter/ActionGraph in the stage
    (Parcel_Sorting_Map has one at each of the A/B/C regions today) and maps
    the first len(regions) of them, ordered by track name, to the given
    region letters.

    route_box(destination) diverts exactly one region's sorter and sets
    every other configured region to pass-through. A destination that
    matches no configured region (e.g. "D") leaves every sorter
    pass-through, so the box naturally travels to the conveyor's end
    (배송지 오류 section) for Arm #2 to pick up.
    """

    def __init__(self, regions=("A", "B", "C"), sorter_speed: float = 1.0):
        self.regions = tuple(regions)
        self.sorter_speed = float(sorter_speed)
        self.units_by_region = {}
        self.all_units = []

    def setup(self):
        """Discover sorter units and set a known initial (pass-through)
        state on every region sorter."""

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage is not available")

        discovered = []
        for prim in stage.Traverse():
            if prim.GetName() != "ActionGraph":
                continue
            parent = prim.GetParent()
            if parent.GetName() != "Sorter":
                continue
            track = parent.GetParent()
            track_name = track.GetName()
            if not track_name.startswith("ConveyorTrack"):
                continue
            discovered.append((track_name, str(prim.GetPath())))

        discovered.sort(key=lambda item: item[0])
        self.all_units = [
            WheelSorterUnit(name, path) for name, path in discovered
        ]

        if len(self.all_units) < len(self.regions):
            print(
                f"[SORTER] WARNING: found {len(self.all_units)} sorter "
                f"unit(s) but {len(self.regions)} region(s) configured "
                f"{self.regions}"
            )

        self.units_by_region = dict(zip(self.regions, self.all_units))

        for unit in self.units_by_region.values():
            unit.set_reroute(False)
            unit.set_speed(self.sorter_speed)

        extra = self.all_units[len(self.units_by_region):]
        for unit in extra:
            # Role not confirmed yet (e.g. ConveyorTrack_05) -- leave the
            # divert state untouched, only make sure it can move.
            unit.set_speed(self.sorter_speed)

        assigned = {r: u.track_name for r, u in self.units_by_region.items()}
        print(f"[SORTER] region sorters ready: {assigned}")
        if extra:
            print(
                f"[SORTER] {len(extra)} unassigned sorter unit(s) found: "
                f"{[u.track_name for u in extra]} (role not wired yet)"
            )

    def route_box(self, destination: str):
        """Set every region sorter's divert state for one box. Boxes are
        placed onto the conveyor one at a time by the inbound arm, so there
        is no concurrent-box queueing to resolve here yet -- call this once
        per box, right as it is placed."""

        target_unit = self.units_by_region.get(destination)
        for unit in self.units_by_region.values():
            unit.set_reroute(unit is target_unit)

        if target_unit is not None:
            print(
                f"[SORTER] routing box -> {destination} "
                f"({target_unit.track_name})"
            )
        else:
            print(
                f"[SORTER] destination '{destination}' matches no region "
                "sorter -- box continues to the end-of-line 배송지 오류 section"
            )
        return destination
