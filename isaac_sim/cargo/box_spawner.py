"""Spawn demo CardBox parcels tagged with a custom ``box_id``.

This module intentionally stays small.  The actual NVIDIA CardBox asset
reference, rigid body, mass, collider, non-uniform scale, and box_id authoring
are handled by ``cargo_pod_physics.add_parcel_asset_scaled()``, which is
already used by the current project.
"""

from cargo.cargo_pod_physics import add_parcel_asset_scaled


def spawn_box_with_id(
    stage,
    prim_path,
    position,
    box_id,
    asset_url,
    scale_xyz=(0.75, 0.75, 0.5),
    mass_kg=15.0,
):
    """Spawn one dynamic NVIDIA CardBox parcel with a custom integer box_id."""

    add_parcel_asset_scaled(
        stage,
        prim_path,
        asset_url=asset_url,
        center=position,
        scale_xyz=scale_xyz,
        box_id=int(box_id),
        mass_kg=float(mass_kg),
    )

    print(
        f"[BOX] spawned {prim_path} at {tuple(position)} "
        f"box_id={int(box_id)} scale={tuple(scale_xyz)}"
    )

    return stage.GetPrimAtPath(prim_path)
