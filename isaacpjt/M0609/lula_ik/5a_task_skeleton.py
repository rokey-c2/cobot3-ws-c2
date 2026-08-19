"""
Task 라이프사이클 — 누가 언제 부르는가

    isaac_python 5a_task_skeleton.py

로봇도 USD 도 없다. print 만 넣어 호출 순서를 확인한다.
World 가 내 메서드를 부르는 시점을 눈으로 본다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import time

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask


PRINT_STEPS = 3     # pre_step 을 몇 번까지 찍을지


class SkeletonTask(BaseTask):
    """
    BaseTask 가 정한 메서드 이름들이다.
    이름을 바꾸면 World 가 찾지 못해 호출되지 않는다.
    """

    def __init__(self, name):
        print("[Task] __init__")
        super().__init__(name=name, offset=None)
        self.step_count = 0

    def set_up_scene(self, scene):
        # world.reset() 안에서 자동으로 불린다
        print("[Task] set_up_scene")
        super().set_up_scene(scene)

    def post_reset(self):
        # set_up_scene 다음, 역시 world.reset() 안에서 불린다
        print("[Task] post_reset")

    def pre_step(self, control_index, simulation_time):
        # world.step() 을 부를 때마다 불린다
        if self.step_count < PRINT_STEPS:
            print(f"[Task] pre_step  {self.step_count}")
        self.step_count += 1

    def get_observations(self):
        # 이것만 내가 직접 부른다
        return {"step": self.step_count}


def main():
    print("\n[main] World()")
    world = World(stage_units_in_meters=1.0)

    print("[main] SkeletonTask()")
    task = SkeletonTask(name="skeleton")

    print("[main] add_task()")
    world.add_task(task)

    print("[main] reset()")
    world.reset()

    print(f"[main] step() x {PRINT_STEPS}")
    for _ in range(PRINT_STEPS):
        world.step(render=True)

    print("[main] get_observations()")
    print(f"       {task.get_observations()}")

    print("\n[main] 확인이 끝나면 창을 닫는다\n")
    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.01)

    simulation_app.close()


if __name__ == "__main__":
    main()