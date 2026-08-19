from pathlib import Path

import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import SingleArticulation


class RMPFlowController(mg.MotionPolicyController):
    """M0609용 RMPFlow controller."""

    def __init__(
        self,
        name: str,
        robot_articulation: SingleArticulation,
        physics_dt: float = 1.0 / 60.0,
        urdf_path: str | None = None,
        robot_description_path: str | None = None,
        rmpflow_config_path: str | None = None,
        end_effector_frame_name: str = "link_6",
        maximum_substep_size: float = 0.00334,
    ) -> None:
        base_dir = Path(__file__).resolve().parent
        urdf_path = str(Path(urdf_path) if urdf_path else base_dir / "m0609_isaac_sim.urdf")
        robot_description_path = str(
            Path(robot_description_path) if robot_description_path else base_dir / "m0609_description.yaml"
        )
        rmpflow_config_path = str(
            Path(rmpflow_config_path) if rmpflow_config_path else base_dir / "m0609_rmpflow_common.yaml"
        )

        self.rmp_flow = mg.lula.motion_policies.RmpFlow(
            robot_description_path=robot_description_path,
            rmpflow_config_path=rmpflow_config_path,
            urdf_path=urdf_path,
            end_effector_frame_name=end_effector_frame_name,
            maximum_substep_size=maximum_substep_size,
        )

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)
        super().__init__(name=name, articulation_motion_policy=self.articulation_rmp)

        self._default_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )

    def reset(self):
        super().reset()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )
