from isaacsim.core.prims import SingleArticulation


class ForkliftBAgent:
    """ForkliftB를 Isaac Sim Articulation으로 관리한다."""

    def __init__(
        self,
        world,
        prim_path="/World/forklift_b_sensor",
        name="forklift_b",
    ):
        self.prim_path = prim_path

        self.robot = world.scene.add(
            SingleArticulation(
                prim_path=self.prim_path,
                name=name,
            )
        )

    def print_joint_info(self):
        print("\n[FORKLIFT] Articulation registered")
        print(f"[FORKLIFT] Prim path: {self.prim_path}")
        print(f"[FORKLIFT] DOF count: {self.robot.num_dof}")

        print("[FORKLIFT] DOF names:")
        for index, joint_name in enumerate(self.robot.dof_names):
            print(f"  {index}: {joint_name}")