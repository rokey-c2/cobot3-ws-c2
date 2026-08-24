from robots.base_robot import BaseRobotAgent


class P3020Agent(BaseRobotAgent):
    """Doosan P3020 공통 Agent."""

    def setup(self):
        # TODO: P3020 + suction gripper load
        pass

    def post_reset(self):
        # TODO: motion controller reset
        pass

    def on_physics_step(self, dt: float):
        # TODO: pick & place state machine
        pass
