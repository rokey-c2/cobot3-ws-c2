ISAAC_SIM_VERSION = "5.1.0"
ROS_DISTRO = "jazzy"
ROS_DOMAIN_ID = 110
RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"

HEADLESS = False

# Physics and rendering both at 60 Hz (Isaac's default). These constants were
# defined but never passed to World(...) before; they are now, kept at 60/60.
# A slower rendering_dt (1/30) was tried: it makes the sim advance ~0.75x
# real time instead of ~0.5x (less laggy to watch) but the full Isaac + Nav2
# + pose-sync pipeline was verified working at 60/60 -- AMCL localizes, the
# NVIDIA IW Hub 2D LiDARs run ~5 Hz in sim time either way (they are self
# limited, not render-tick limited), GPU sits at ~55% with one sim instance.
# Bump RENDERING_DT to 1/30 if a snappier viewport matters more than staying
# on Isaac's default. The real non-headless wins are the RTX quality settings
# in apply_render_optimizations() and not running two sim instances at once.
PHYSICS_DT = 1.0 / 60.0
RENDERING_DT = 1.0 / 60.0

# Passed to SimulationApp(...). Keeps the RTX renderer in real-time
# (raytraced) mode instead of path tracing, caps render resolution, turns on
# DLSS, and trims ray-bounce budgets -- all things that otherwise default
# high and stall a 16 GB laptop GPU on this asset-heavy map.
LAUNCH_CONFIG = {
    "headless": HEADLESS,
    # Main RTX viewport render size. This is separate from the P3020 camera
    # render products, so shrinking it only costs viewport sharpness, not
    # detection input. 960x540 keeps it usable while cutting the per-frame
    # RTX pass the sim thread waits on.
    "width": 960,
    "height": 540,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 3,  # DLSS
    "samples_per_pixel_per_frame": 1,
    "max_bounces": 2,
    "max_specular_transmission_bounces": 2,
    "max_volume_bounces": 0,
    "subdiv_refinement_level": 0,
}

# None -> CameraInterface derives the pixel grid from the camera prim's
# AUTHORED horizontalAperture/verticalAperture so the streamed/detected image
# matches the angle set in the viewport exactly (no crop, no aperture
# rewrite). Height defaults to 480 (env P3020_CAMERA_HEIGHT); lower it once
# detection works if the extra pixels cost too much. Set an explicit (w, h)
# tuple here only to override the derivation.
P3020_CAMERA_RESOLUTION = None

# Cameras with zero code references anywhere in the project -- the extra RGB
# sensors on each P3020 RSD455 rig. The pick/place + YOLO pipeline only ever
# touches Camera_Pseudo_Depth (see CAMERA_PRIM_PATH in p3020_mission_agent.py
# / p3020_out_mission_agent.py), which serves both /rgb and the depth AOV, so
# it is deliberately NOT listed here and stays active.
#
# The IW Hub AMR cameras (transporter_camera_first_person / _third_person,
# camera_left, camera_right) are intentionally left active: a camera prim
# with no render product is essentially free, and the front view is wanted
# for a future dashboard feed. Add a name here to deactivate one.
_UNUSED_CAMERA_NAMES = (
    "Camera_OmniVision_OV9782_Color",
    "Camera_OmniVision_OV9782_Left",
    "Camera_OmniVision_OV9782_Right",
)


def apply_render_optimizations():
    """Mirror the GUI Render Settings tweaks via carb settings so every run
    starts optimized instead of needing the window touched by hand.

    Safe to call after simulation_app.update(). Never raises -- a missing or
    renamed setting key is just skipped."""

    try:
        import carb

        settings = carb.settings.get_settings()
    except Exception as exc:  # pragma: no cover - only in a real Isaac run
        print(f"[PERF] carb settings unavailable, skipping render tuning: {exc!r}")
        return

    values = {
        "/rtx/rendermode": "RaytracedLighting",
        "/rtx/post/aa/op": 3,  # DLSS
        "/rtx/post/dlss/execMode": 0,  # Max Performance
        "/rtx/reflections/enabled": False,
        "/rtx/translucency/enabled": False,
        "/rtx/ambientOcclusion/enabled": False,
        "/rtx/indirectDiffuse/enabled": False,
        "/rtx/directLighting/sampledLighting/enabled": True,
        "/rtx-defaults/resourcemanager/enableTextureStreaming": True,
        "/rtx/resourcemanager/texturestreaming/memoryBudget": 0.25,
        "/app/runLoops/main/rateLimitEnabled": True,
        "/app/runLoops/main/rateLimitFrequency": 30,
        # Skip re-rendering a frame when nothing in view changed. During
        # long idle waits (AMR driving off-camera, arm parked) this drops
        # the RTX pass entirely; it re-enables itself the moment the scene
        # moves, so it does not slow the actual mission.
        "/rtx/ecoMode/enabled": True,
    }

    applied = 0
    for key, value in values.items():
        try:
            settings.set(key, value)
            applied += 1
        except Exception as exc:  # pragma: no cover
            print(f"[PERF] could not set {key}: {exc!r}")

    print(f"[PERF] render optimizations applied ({applied}/{len(values)} settings)")


def disable_unused_cameras(stage):
    """Deactivate viewport cameras nothing subscribes to, to drop them from
    the render graph. Uses SetActive(False): reversible and never writes the
    saved map file. FPS gain is small unless a camera had a render product,
    but it costs nothing and declutters the viewport dropdown."""

    try:
        from pxr import UsdGeom
    except Exception as exc:  # pragma: no cover
        print(f"[PERF] pxr unavailable, skipping camera cleanup: {exc!r}")
        return

    disabled = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Camera):
            continue
        if prim.GetName() not in _UNUSED_CAMERA_NAMES:
            continue
        if not prim.IsActive():
            continue
        prim.SetActive(False)
        disabled.append(prim.GetPath().pathString)

    if disabled:
        print(f"[PERF] disabled {len(disabled)} unused cameras: {disabled}")
    else:
        print(
            "[PERF] no unused cameras matched "
            f"{_UNUSED_CAMERA_NAMES} -- map structure may have changed"
        )
