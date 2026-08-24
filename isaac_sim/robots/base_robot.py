from abc import ABC, abstractmethod


class BaseRobotAgent(ABC):
    def __init__(self, cfg: dict, world=None):
        self.cfg = cfg
        self.world = world
        self.name = cfg["name"]
        self.namespace = cfg["namespace"]
        self.role = cfg["role"]

    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def post_reset(self):
        pass

    @abstractmethod
    def on_physics_step(self, dt: float):
        pass

    def on_render_step(self):
        pass
