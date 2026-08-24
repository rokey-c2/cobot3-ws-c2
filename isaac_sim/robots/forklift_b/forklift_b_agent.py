from robots.base_robot import BaseRobotAgent


class ForkliftBAgent(BaseRobotAgent):
    """ForkliftB 공통 Agent.

    하나의 Agent 클래스로 6대의 ForkliftB 인스턴스를 생성합니다.
    """

    def setup(self):
        # TODO: ForkliftB asset/articulation load
        pass

    def post_reset(self):
        # TODO: controller reset
        pass

    def on_physics_step(self, dt: float):
        # TODO: navigation / fork control
        pass
