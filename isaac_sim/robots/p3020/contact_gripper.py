"""VGP20용 "닿으면 붙는다" 방식의 임시 흡착 구현.

SurfaceGripper(레이캐스트 기반)는 방향/위치를 다 정확히 맞췄는데도
안정적으로 안 붙어서, 대신 훨씬 단순하고 예측 가능한 방식을 쓴다:

    매 스텝마다 흡착 컵 위치(local_pos, 이번 세션에서 실측/검증된 값)와
    박스 사이의 실제 거리를 재서, threshold 안에 들어오면 그 순간
    PhysicsFixedJoint 로 그리퍼와 박스를 용접한다.

VGP20 하드웨어(모델)는 그대로 쓰되, 흡착의 "판정 로직"만 대체하는 것.
"""

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf


class ContactGripper:
    def __init__(
        self,
        stage,
        gripper_body_path: str,
        local_pos: Gf.Vec3f,
        contact_threshold: float = 0.03,
        joint_path: str = None,
        snap_distance: float = 0.08,
        local_down_dir: Gf.Vec3d = Gf.Vec3d(0, -1, 0),
    ):
        self._stage = stage
        self._gripper_body_path = gripper_body_path
        self._local_pos = local_pos
        self._threshold = contact_threshold
        self._joint_path = joint_path or f"{gripper_body_path}_contact_joint"
        self._attached_to = None
        # 콜리전을 꺼놨기 때문에, 붙는 순간의 우연한(겹친) 위치 그대로 용접되면
        # 시각적으로 박스가 그리퍼를 뚫고 겹쳐 보인다. 그래서 용접 직전에 박스를
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
        그 순간 용접한다. 붙었는지 여부를 반환한다."""
        if self._attached_to is not None:
            return True
        if self.distance_to(object_prim_path) <= self._threshold:
            self._create_joint(object_prim_path)
            self._attached_to = object_prim_path
            return True
        return False

    def _local_snap_target(self) -> Gf.Vec3d:
        """vgp20 로컬 좌표계 기준, 흡착 컵 바로 아래(박스가 매달릴) 지점."""
        return (
            Gf.Vec3d(self._local_pos[0], self._local_pos[1], self._local_pos[2])
            + self._local_down_dir * self._snap_distance
        )

    def _set_object_translate(self, object_prim_path: str, world_pos: Gf.Vec3d):
        xformable = UsdGeom.Xformable(self._stage.GetPrimAtPath(object_prim_path))
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(world_pos)
                return
        xformable.AddTranslateOp().Set(world_pos)

    def _create_joint(self, object_prim_path: str):
        vgp20_xf = UsdGeom.Xformable(self._stage.GetPrimAtPath(self._gripper_body_path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        local_target = self._local_snap_target()

        # 화면에 보이는 위치도 미리 스냅해두고(물리가 아직 안 도는 순간에도 깔끔하게
        # 보이도록), 조인트에도 같은 상대 위치를 localPos0/localPos1로 명시한다.
        # localPos를 안 주면 PhysicsFixedJoint는 두 바디의 "원점"을 강제로 일치시켜
        # 버려서(0,0,0 기본값), 스냅한 위치가 무시되고 그리퍼 원점으로 끌려간다.
        self._set_object_translate(object_prim_path, vgp20_xf.Transform(local_target))

        if self._stage.GetPrimAtPath(self._joint_path).IsValid():
            self._stage.RemovePrim(self._joint_path)
        joint_prim = self._stage.DefinePrim(self._joint_path, "PhysicsFixedJoint")
        joint = UsdPhysics.FixedJoint(joint_prim)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(self._gripper_body_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(object_prim_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(local_target))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateJointEnabledAttr().Set(True)
        joint.CreateBreakForceAttr().Set(3.4028235e38)
        joint.CreateBreakTorqueAttr().Set(3.4028235e38)
        joint.CreateExcludeFromArticulationAttr().Set(True)

    def detach(self):
        prim = self._stage.GetPrimAtPath(self._joint_path)
        if prim.IsValid():
            self._stage.RemovePrim(self._joint_path)
        self._attached_to = None

    def is_attached(self) -> bool:
        return self._attached_to is not None

    def gripped_object(self):
        return self._attached_to
