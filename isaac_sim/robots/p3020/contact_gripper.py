"""VGP20용 "닿으면 붙는다" 방식의 임시 흡착 구현.

SurfaceGripper(레이캐스트 기반)는 방향/위치를 다 정확히 맞췄는데도
안정적으로 안 붙어서, 대신 훨씬 단순하고 예측 가능한 방식을 쓴다:

    매 스텝마다 흡착 컵 위치(local_pos, 이번 세션에서 실측/검증된 값)와
    박스 사이의 실제 거리를 재서, threshold 안에 들어오면 그 순간 붙잡는다.

처음엔 PhysicsFixedJoint 로 그리퍼와 박스를 용접했었는데, 진단 결과
(diag_gap.py) 그 방식은 물리 솔버가 몇 스텝 지나면서 의도한 위치에서
3~5cm씩 어긋나는 것으로 확인됐다 (박스가 그리퍼 쪽으로 파고들거나 반대로
뜨는 것처럼 보였던 원인). vgp20이 로봇 팔의 관절 좌표계에 물려있는 링크라서,
거기에 별도의 maximal-coordinate 조인트(박스용)를 하나 더 얹으면 두 종류의
솔버가 완벽히 안 맞아떨어지는 듯하다.

그래서 조인트 대신, 붙어있는 동안은 박스를 kinematic으로 전환하고 매 스텝
"그리퍼 기준 고정 오프셋" 위치로 직접 트랜스폼을 덮어쓰는 방식으로 바꿨다.
이러면 물리 솔버의 수렴 오차 없이 항상 정확히 같은 상대 위치를 유지한다.
놓을 때는 다시 dynamic으로 돌려서 중력으로 자연스럽게 낙하시킨다.

VGP20 하드웨어(모델)는 그대로 쓰되, 흡착의 "판정/유지 로직"만 대체하는 것.
"""

from pxr import Usd, UsdGeom, UsdPhysics, Gf


class ContactGripper:
    def __init__(
        self,
        stage,
        gripper_body_path: str,
        local_pos: Gf.Vec3f,
        contact_threshold: float = 0.03,
        snap_distance: float = 0.08,
        local_down_dir: Gf.Vec3d = Gf.Vec3d(0, -1, 0),
    ):
        self._stage = stage
        self._gripper_body_path = gripper_body_path
        self._local_pos = local_pos
        self._threshold = contact_threshold
        self._attached_to = None
        # 콜리전을 꺼놨기 때문에, 붙는 순간의 우연한(겹친) 위치 그대로 잡으면
        # 시각적으로 박스가 그리퍼를 뚫고 겹쳐 보인다. 그래서 잡는 순간 박스를
        # 흡착 컵 바로 아래(local_pos에서 local_down_dir 방향으로 snap_distance
        # 만큼 떨어진 지점)로 깔끔하게 스냅시킨다.
        self._snap_distance = snap_distance
        self._local_down_dir = Gf.Vec3d(local_down_dir).GetNormalized()

    def gripper_point_world(self) -> Gf.Vec3d:
        xf = UsdGeom.Xformable(self._stage.GetPrimAtPath(self._gripper_body_path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        return xf.Transform(Gf.Vec3d(self._local_pos[0], self._local_pos[1], self._local_pos[2]))

    def distance_to(self, object_prim_path: str) -> float:
        obj_xf = UsdGeom.Xformable(self._stage.GetPrimAtPath(object_prim_path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        obj_pos = obj_xf.Transform(Gf.Vec3d(0, 0, 0))
        return (self.gripper_point_world() - obj_pos).GetLength()

    def try_attach(self, object_prim_path: str) -> bool:
        """매 스텝 호출. 이미 붙어 있으면 그대로 유지, threshold 안에 들어오면
        그 순간 잡는다. 붙었는지 여부를 반환한다."""
        if self._attached_to is not None:
            return True
        if self.distance_to(object_prim_path) <= self._threshold:
            self._attach(object_prim_path)
            return True
        return False

    def _local_snap_target(self) -> Gf.Vec3d:
        """vgp20 로컬 좌표계 기준, 흡착 컵 바로 아래(박스 원점이 있어야 할) 지점."""
        return (
            Gf.Vec3d(self._local_pos[0], self._local_pos[1], self._local_pos[2])
            + self._local_down_dir * self._snap_distance
        )

    def _attach(self, object_prim_path: str):
        prim = self._stage.GetPrimAtPath(object_prim_path)
        # kinematic으로 바꿔서 이제부턴 물리(중력/충돌)가 아니라 우리가 매 스텝
        # 직접 트랜스폼을 써주는 방식으로 위치를 고정한다 -- 조인트 솔버 오차가
        # 없어서 그리퍼-박스 사이 간격이 절대 벌어지거나 파고들지 않는다.
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
        self._attached_to = object_prim_path
        self.update()

    def update(self):
        """매 물리 스텝마다 (붙어있는 동안) 호출해야 한다. 박스 위치를 그리퍼
        기준 고정 오프셋으로 매번 다시 스냅해서, 팔이 움직이는 동안에도 간격이
        벌어지거나 겹치지 않게 유지한다.

        회전(orient)은 일부러 건드리지 않는다 -- vgp20 자체가 "컵이 아래를
        보게" 하려고 항상 기울어진 자세이기 때문에, 그 회전을 그대로 박스에
        복사하면 박스가 원래(바닥에 놓였을 때의 반듯한) 자세를 잃고 그리퍼와
        같이 기울어져 버린다 (놓았을 때 옆으로 넘어져 버리는 원인이었다).
        박스는 항상 원래 자세를 유지한 채 위치만 따라간다."""
        if self._attached_to is None:
            return
        gripper_xf = UsdGeom.Xformable(self._stage.GetPrimAtPath(self._gripper_body_path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        world_pos = gripper_xf.Transform(self._local_snap_target())
        self._set_object_translate(self._attached_to, world_pos)

    def _set_object_translate(self, object_prim_path: str, world_pos: Gf.Vec3d):
        """object의 부모가 /World(단위 트랜스폼)라고 가정하고, world 위치를 그대로
        local translate 로 authoring 한다 (이 프로젝트의 TargetBox는 항상 /World
        바로 아래라 이 가정이 성립한다)."""
        xformable = UsdGeom.Xformable(self._stage.GetPrimAtPath(object_prim_path))
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(world_pos)
                return
        xformable.AddTranslateOp().Set(world_pos)

    def detach(self):
        if self._attached_to is not None:
            prim = self._stage.GetPrimAtPath(self._attached_to)
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(False)
        self._attached_to = None

    def is_attached(self) -> bool:
        return self._attached_to is not None

    def gripped_object(self):
        return self._attached_to
