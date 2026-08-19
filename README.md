# cobot3-ws-c2
def _disable_camera_rigid_body(self):
    """브래킷 아래에 중첩된 카메라 Rigid Body를 비활성화한다."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(CAMERA_BODY_PRIM_PATH)

    if not prim.IsValid():
        print(
            f"   camera body  not found: "
            f"{CAMERA_BODY_PRIM_PATH}"
        )
        return

    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        print("   camera body  RigidBodyAPI not applied")
        return

    rigid_body = UsdPhysics.RigidBodyAPI(prim)
    rigid_body.GetRigidBodyEnabledAttr().Set(False)

    print(
        "   camera body  nested RigidBody disabled: "
        f"{CAMERA_BODY_PRIM_PATH}"
    )
