from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import random
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.utils.semantics import add_labels
from isaacsim.storage.native import get_assets_root_path
from pxr import Sdf, UsdGeom, UsdPhysics

assets_root_path = get_assets_root_path()
stage = omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()

# 조명
light = stage.DefinePrim("/World/Lights/DistantLight", "DistantLight")
light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(1000.0)

# 바닥 (컨베이어/테이블 대용 평면)
ground = stage.DefinePrim("/World/Ground", "Cube")
UsdGeom.Xformable(ground).AddScaleOp().Set((2.0, 2.0, 0.02))
UsdGeom.Xformable(ground).AddTranslateOp().Set((0, 0, -0.01))

# 박스 에셋 (택배 상자 역할) - 여러 개 생성
box_urls = [
    "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01_414.usd",
    "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04_1847.usd",
]
box_prims = []
for i in range(5):
    prim_path = f"/World/Boxes/box_{i}"
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(assets_root_path + random.choice(box_urls))
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set((random.uniform(-0.3, 0.3), random.uniform(-0.3, 0.3), 0.05))
    xf.AddRotateXYZOp().Set((0, 0, random.uniform(0, 360)))
    add_labels(prim, labels=["box"], instance_name="class")
    box_prims.append(prim)

# 카메라 (실제 그리퍼 장착 위치를 흉내낸 고정 시점 - 나중에 실측값으로 교체)
cam = stage.DefinePrim("/World/Camera", "Camera")
UsdGeom.Xformable(cam).AddTranslateOp().Set((0, 0, 0.5))
UsdGeom.Xformable(cam).AddRotateXYZOp().Set((-90, 0, 0))  # 아래를 내려다보게

simulation_app.update()

# Render Product + Writer
rp = rep.create.render_product(cam.GetPath(), (640, 480))
writer = rep.writers.get("BasicWriter")
writer.initialize(
    output_dir="_out_parcel_sdg",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=False,
)
writer.attach([rp])

rep.orchestrator.set_capture_on_play(False)

num_frames = 100
for i in range(num_frames):
    # 매 프레임 박스 위치/회전 랜덤화
    for prim in box_prims:
        xf = UsdGeom.Xformable(prim)
        ops = xf.GetOrderedXformOps()
        ops[0].Set((random.uniform(-0.3, 0.3), random.uniform(-0.3, 0.3), 0.05))
        ops[1].Set((0, 0, random.uniform(0, 360)))
    simulation_app.update()
    rep.orchestrator.step()

rep.orchestrator.wait_until_complete()
simulation_app.close()