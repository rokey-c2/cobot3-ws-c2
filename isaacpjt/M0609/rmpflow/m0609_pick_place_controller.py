from typing import List, Optional

import isaacsim.robot.manipulators.controllers as manipulators_controllers
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper

from m0609_rmpflow_controller import RMPFlowController


class PickPlaceController(manipulators_controllers.PickPlaceController):
    """M0609용 pick&place controller."""

    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: SingleArticulation,
        end_effector_initial_height: Optional[float] = None,
        events_dt: Optional[List[float]] = None,
        urdf_path: str | None = None,
        robot_description_path: str | None = None,
        rmpflow_config_path: str | None = None,
        end_effector_frame_name: str = "link_6",
    ) -> None:
        if events_dt is None:
            events_dt = [0.008, 0.005, 1.0, 0.1, 0.05, 0.05, 0.0025, 1.0, 0.008, 0.08]

        super().__init__(
            name=name,
            cspace_controller=RMPFlowController(
                name=name + "_cspace_controller",
                robot_articulation=robot_articulation,
                urdf_path=urdf_path,
                robot_description_path=robot_description_path,
                rmpflow_config_path=rmpflow_config_path,
                end_effector_frame_name=end_effector_frame_name,
            ),
            gripper=gripper,
            end_effector_initial_height=end_effector_initial_height,
            events_dt=events_dt,
        )