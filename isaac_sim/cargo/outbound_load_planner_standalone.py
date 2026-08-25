#!/usr/bin/env python3
"""Standalone 3x3 outbound AMR load planner.

This file is intentionally independent from ROS 2, Isaac Sim, and the project
repository.  It models one outbound station that places nine equal-size boxes
on a single layer of an AMR cargo deck.

Run the example:
    python3 outbound_load_planner_standalone.py --demo

Run the built-in unit tests:
    python3 outbound_load_planner_standalone.py --test
"""

from __future__ import annotations

import argparse
import math
import unittest
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


XY = Tuple[float, float]
XYZ = Tuple[float, float, float]


@dataclass(frozen=True)
class PlannerConfig:
    """Geometry and clearance values in metres."""

    rows: int = 3
    columns: int = 3
    box_length: float = 0.30
    box_width: float = 0.30
    box_height: float = 0.30
    gap_x: float = 0.01
    gap_y: float = 0.01
    usable_deck_length: float = 0.96
    usable_deck_width: float = 0.96
    guard_height: float = 0.12
    place_clearance: float = 0.005
    transit_clearance: float = 0.20

    def validate(self) -> None:
        positive_values = {
            "rows": self.rows,
            "columns": self.columns,
            "box_length": self.box_length,
            "box_width": self.box_width,
            "box_height": self.box_height,
            "usable_deck_length": self.usable_deck_length,
            "usable_deck_width": self.usable_deck_width,
        }
        for name, value in positive_values.items():
            if value <= 0:
                raise ValueError(f"{name} must be positive")

        non_negative_values = {
            "gap_x": self.gap_x,
            "gap_y": self.gap_y,
            "guard_height": self.guard_height,
            "place_clearance": self.place_clearance,
            "transit_clearance": self.transit_clearance,
        }
        for name, value in non_negative_values.items():
            if value < 0:
                raise ValueError(f"{name} cannot be negative")

        required_length = (
            self.columns * self.box_length
            + (self.columns - 1) * self.gap_x
        )
        required_width = (
            self.rows * self.box_width
            + (self.rows - 1) * self.gap_y
        )
        if required_length > self.usable_deck_length + 1.0e-9:
            raise ValueError(
                f"boxes need {required_length:.3f} m of deck length, "
                f"but only {self.usable_deck_length:.3f} m is usable"
            )
        if required_width > self.usable_deck_width + 1.0e-9:
            raise ValueError(
                f"boxes need {required_width:.3f} m of deck width, "
                f"but only {self.usable_deck_width:.3f} m is usable"
            )

    @property
    def capacity(self) -> int:
        return self.rows * self.columns

    @property
    def pitch_x(self) -> float:
        return self.box_length + self.gap_x

    @property
    def pitch_y(self) -> float:
        return self.box_width + self.gap_y


@dataclass(frozen=True)
class CargoPose:
    """Cargo-deck centre pose in the world frame.

    floor_z is the Z coordinate of the top surface on which boxes are placed.
    yaw_deg is the cargo deck's counter-clockwise rotation around world Z.
    """

    x: float
    y: float
    floor_z: float
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class Slot:
    slot_id: int
    row: int
    column: int
    local_x: float
    local_y: float


@dataclass(frozen=True)
class PlacementPlan:
    amr_id: str
    slot_id: int
    row: int
    column: int
    place_position: XYZ
    approach_position: XYZ
    place_yaw_deg: float
    remaining_after_success: int
    full_after_success: bool


class OutboundLoadPlanner:
    """Transactional single-layer load planner for one outbound station.

    A slot is not considered occupied when it is merely planned.  The caller
    must call confirm_placement() after the gripper has released the box and
    the placement has been verified.  On a failed attempt, call
    cancel_placement() so that the same slot can be retried.
    """

    def __init__(self, config: PlannerConfig = PlannerConfig()):
        config.validate()
        self.config = config
        self._slots = tuple(self._build_slots())
        self._amr_id: Optional[str] = None
        self._occupied: set[int] = set()
        self._pending: Optional[PlacementPlan] = None

    def _build_slots(self) -> Iterable[Slot]:
        x_mid = (self.config.columns - 1) / 2.0
        y_mid = (self.config.rows - 1) / 2.0
        slot_id = 0
        for row in range(self.config.rows):
            for column in range(self.config.columns):
                yield Slot(
                    slot_id=slot_id,
                    row=row,
                    column=column,
                    local_x=(column - x_mid) * self.config.pitch_x,
                    local_y=(row - y_mid) * self.config.pitch_y,
                )
                slot_id += 1

    def start_amr(self, amr_id: str) -> None:
        """Attach a newly docked AMR and reset its nine-slot load state."""

        clean_id = amr_id.strip()
        if not clean_id:
            raise ValueError("amr_id cannot be empty")
        # Docking/status topics are usually periodic. Repeated notification for
        # the same vehicle must not erase its already-confirmed load state.
        if clean_id == self._amr_id:
            return
        if self._pending is not None:
            raise RuntimeError(
                "cannot change AMR while a placement is pending; "
                "confirm or cancel it first"
            )
        self._amr_id = clean_id
        self._occupied.clear()

    @property
    def amr_id(self) -> Optional[str]:
        return self._amr_id

    @property
    def occupied_count(self) -> int:
        return len(self._occupied)

    @property
    def remaining_count(self) -> int:
        return self.config.capacity - len(self._occupied)

    @property
    def is_full(self) -> bool:
        return self.occupied_count == self.config.capacity

    @property
    def occupied_slot_ids(self) -> Tuple[int, ...]:
        return tuple(sorted(self._occupied))

    @staticmethod
    def _local_to_world(local_xy: XY, cargo_pose: CargoPose) -> XY:
        yaw = math.radians(cargo_pose.yaw_deg)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x, local_y = local_xy
        return (
            cargo_pose.x + cos_yaw * local_x - sin_yaw * local_y,
            cargo_pose.y + sin_yaw * local_x + cos_yaw * local_y,
        )

    def slot_world_xy(self, slot: Slot, cargo_pose: CargoPose) -> XY:
        return self._local_to_world(
            (slot.local_x, slot.local_y), cargo_pose
        )

    def all_slot_world_positions(self, cargo_pose: CargoPose) -> Tuple[XYZ, ...]:
        place_z = (
            cargo_pose.floor_z
            + self.config.box_height / 2.0
            + self.config.place_clearance
        )
        return tuple(
            (*self.slot_world_xy(slot, cargo_pose), place_z)
            for slot in self._slots
        )

    def _ordered_available_slots(
        self, cargo_pose: CargoPose, arm_xy: XY
    ) -> list[Slot]:
        available = [
            slot
            for slot in self._slots
            if slot.slot_id not in self._occupied
        ]

        def sort_key(slot: Slot) -> tuple[float, int, int]:
            world_x, world_y = self.slot_world_xy(slot, cargo_pose)
            distance_sq = (
                (world_x - arm_xy[0]) ** 2
                + (world_y - arm_xy[1]) ** 2
            )
            # Negative distance means farthest first. Row/column make ties
            # deterministic for repeatable tests and simulation runs.
            return (-distance_sq, slot.row, slot.column)

        return sorted(available, key=sort_key)

    def plan_next(self, cargo_pose: CargoPose, arm_xy: XY) -> PlacementPlan:
        """Lock and return the next far-to-near placement target."""

        if self._amr_id is None:
            raise RuntimeError("no AMR is attached; call start_amr() first")
        if self._pending is not None:
            return self._pending
        if self.is_full:
            raise RuntimeError("AMR is full")

        slot = self._ordered_available_slots(cargo_pose, arm_xy)[0]
        world_x, world_y = self.slot_world_xy(slot, cargo_pose)
        place_z = (
            cargo_pose.floor_z
            + self.config.box_height / 2.0
            + self.config.place_clearance
        )
        obstacle_top_z = cargo_pose.floor_z + max(
            self.config.box_height, self.config.guard_height
        )
        transit_z = obstacle_top_z + self.config.transit_clearance
        remaining = self.remaining_count - 1

        self._pending = PlacementPlan(
            amr_id=self._amr_id,
            slot_id=slot.slot_id,
            row=slot.row,
            column=slot.column,
            place_position=(world_x, world_y, place_z),
            approach_position=(world_x, world_y, transit_z),
            place_yaw_deg=cargo_pose.yaw_deg,
            remaining_after_success=remaining,
            full_after_success=(remaining == 0),
        )
        return self._pending

    def confirm_placement(self, slot_id: int) -> None:
        """Commit a slot only after successful release/place verification."""

        if self._pending is None:
            raise RuntimeError("there is no pending placement")
        if slot_id != self._pending.slot_id:
            raise ValueError(
                f"pending slot is {self._pending.slot_id}, not {slot_id}"
            )
        self._occupied.add(slot_id)
        self._pending = None

    def cancel_placement(self, slot_id: int) -> None:
        """Release a planned slot after a failed pick or placement."""

        if self._pending is None:
            raise RuntimeError("there is no pending placement")
        if slot_id != self._pending.slot_id:
            raise ValueError(
                f"pending slot is {self._pending.slot_id}, not {slot_id}"
            )
        self._pending = None


class OutboundLoadPlannerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = PlannerConfig()
        self.planner = OutboundLoadPlanner(self.config)
        self.cargo = CargoPose(x=10.0, y=5.0, floor_z=0.30, yaw_deg=0.0)

    def test_default_geometry_fits_nine_unique_slots(self) -> None:
        positions = self.planner.all_slot_world_positions(self.cargo)
        self.assertEqual(len(positions), 9)
        self.assertEqual(len(set(positions)), 9)
        self.assertAlmostEqual(max(p[0] for p in positions), 10.31)
        self.assertAlmostEqual(min(p[0] for p in positions), 9.69)
        self.assertAlmostEqual(max(p[1] for p in positions), 5.31)
        self.assertAlmostEqual(min(p[1] for p in positions), 4.69)

    def test_every_box_footprint_stays_inside_usable_deck(self) -> None:
        positions = self.planner.all_slot_world_positions(self.cargo)
        deck_half_length = self.config.usable_deck_length / 2.0
        deck_half_width = self.config.usable_deck_width / 2.0
        box_half_length = self.config.box_length / 2.0
        box_half_width = self.config.box_width / 2.0

        for x, y, _ in positions:
            self.assertLessEqual(
                abs(x - self.cargo.x) + box_half_length,
                deck_half_length + 1.0e-9,
            )
            self.assertLessEqual(
                abs(y - self.cargo.y) + box_half_width,
                deck_half_width + 1.0e-9,
            )

    def test_invalid_deck_size_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            OutboundLoadPlanner(
                PlannerConfig(usable_deck_length=0.90)
            )

    def test_ninety_degree_cargo_rotation_rotates_slots(self) -> None:
        rotated = CargoPose(x=0.0, y=0.0, floor_z=0.30, yaw_deg=90.0)
        positions = self.planner.all_slot_world_positions(rotated)
        # Local slot 0 is (-0.31, -0.31). After +90 deg it is (+0.31, -0.31).
        self.assertAlmostEqual(positions[0][0], 0.31, places=9)
        self.assertAlmostEqual(positions[0][1], -0.31, places=9)

    def test_cardinal_cargo_rotations_preserve_nine_unique_slots(self) -> None:
        expected_radii = sorted(
            round(math.hypot(x - self.cargo.x, y - self.cargo.y), 9)
            for x, y, _ in self.planner.all_slot_world_positions(self.cargo)
        )

        for yaw_deg in (0.0, 90.0, 180.0, 270.0):
            cargo = CargoPose(x=10.0, y=5.0, floor_z=0.30, yaw_deg=yaw_deg)
            positions = self.planner.all_slot_world_positions(cargo)
            rounded_xyz = {
                (round(x, 9), round(y, 9), round(z, 9))
                for x, y, z in positions
            }
            radii = sorted(
                round(math.hypot(x - cargo.x, y - cargo.y), 9)
                for x, y, _ in positions
            )
            self.assertEqual(len(rounded_xyz), 9)
            self.assertEqual(radii, expected_radii)

    def test_farthest_slot_is_selected_first(self) -> None:
        self.planner.start_amr("amr_out_a_01")
        plan = self.planner.plan_next(self.cargo, arm_xy=(10.0, 3.0))
        self.assertAlmostEqual(plan.place_position[1], 5.31)

    def test_planning_is_transactional_and_idempotent(self) -> None:
        self.planner.start_amr("amr_out_a_01")
        first = self.planner.plan_next(self.cargo, arm_xy=(10.0, 3.0))
        repeated = self.planner.plan_next(self.cargo, arm_xy=(10.0, 3.0))
        self.assertEqual(first, repeated)
        self.assertEqual(self.planner.occupied_count, 0)

        self.planner.cancel_placement(first.slot_id)
        retry = self.planner.plan_next(self.cargo, arm_xy=(10.0, 3.0))
        self.assertEqual(retry.slot_id, first.slot_id)
        self.planner.confirm_placement(retry.slot_id)
        self.assertEqual(self.planner.occupied_count, 1)

    def test_guard_lower_than_box_uses_box_top_for_transit(self) -> None:
        self.planner.start_amr("amr_out_a_01")
        plan = self.planner.plan_next(self.cargo, arm_xy=(10.0, 3.0))
        expected_place_z = 0.30 + 0.15 + 0.005
        expected_transit_z = 0.30 + 0.30 + 0.20
        self.assertAlmostEqual(plan.place_position[2], expected_place_z)
        self.assertAlmostEqual(plan.approach_position[2], expected_transit_z)

    def test_repository_geometry_has_clearance_over_box_and_low_guard(self) -> None:
        self.planner.start_amr("amr_out_a_01")
        plan = self.planner.plan_next(self.cargo, arm_xy=(10.0, 3.0))
        box_top_z = self.cargo.floor_z + self.config.box_height
        guard_top_z = self.cargo.floor_z + self.config.guard_height

        self.assertAlmostEqual(plan.place_position[2], 0.455)
        self.assertAlmostEqual(box_top_z, 0.600)
        self.assertAlmostEqual(guard_top_z, 0.420)
        self.assertAlmostEqual(plan.approach_position[2], 0.800)
        self.assertGreater(plan.approach_position[2], box_top_z)
        self.assertGreater(plan.approach_position[2], guard_top_z)

    def test_full_after_nine_confirmed_placements(self) -> None:
        self.planner.start_amr("amr_out_a_01")
        for expected_count in range(1, 10):
            plan = self.planner.plan_next(self.cargo, arm_xy=(10.0, 3.0))
            self.planner.confirm_placement(plan.slot_id)
            self.assertEqual(self.planner.occupied_count, expected_count)
        self.assertTrue(self.planner.is_full)
        self.assertEqual(self.planner.remaining_count, 0)
        with self.assertRaises(RuntimeError):
            self.planner.plan_next(self.cargo, arm_xy=(10.0, 3.0))

    def test_new_amr_resets_slots(self) -> None:
        self.planner.start_amr("amr_out_a_01")
        plan = self.planner.plan_next(self.cargo, arm_xy=(10.0, 3.0))
        self.planner.confirm_placement(plan.slot_id)
        self.assertEqual(self.planner.occupied_count, 1)

        self.planner.start_amr("amr_out_a_02")
        self.assertEqual(self.planner.occupied_count, 0)
        self.assertEqual(self.planner.remaining_count, 9)

    def test_repeated_same_amr_signal_preserves_slots(self) -> None:
        self.planner.start_amr("amr_out_a_01")
        plan = self.planner.plan_next(self.cargo, arm_xy=(10.0, 3.0))
        self.planner.confirm_placement(plan.slot_id)

        self.planner.start_amr("amr_out_a_01")
        self.assertEqual(self.planner.occupied_count, 1)
        self.assertEqual(self.planner.remaining_count, 8)

    def test_cannot_change_amr_during_pending_placement(self) -> None:
        self.planner.start_amr("amr_out_a_01")
        self.planner.plan_next(self.cargo, arm_xy=(10.0, 3.0))
        with self.assertRaises(RuntimeError):
            self.planner.start_amr("amr_out_a_02")


def run_demo() -> None:
    planner = OutboundLoadPlanner()
    cargo = CargoPose(x=1.50, y=-0.50, floor_z=0.30, yaw_deg=0.0)
    arm_xy = (1.50, -2.00)
    planner.start_amr("amr_out_a_01")

    print("3x3 single-layer outbound loading demo")
    print(f"AMR: {planner.amr_id}, capacity: {planner.config.capacity}")
    print("Farthest slots from the arm are filled first.\n")

    while not planner.is_full:
        plan = planner.plan_next(cargo, arm_xy)
        x, y, z = plan.place_position
        ax, ay, az = plan.approach_position
        print(
            f"#{planner.occupied_count + 1}: slot={plan.slot_id} "
            f"row={plan.row} col={plan.column} "
            f"place=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"approach=({ax:.3f}, {ay:.3f}, {az:.3f})"
        )
        # In the real system, call this only after release/place verification.
        planner.confirm_placement(plan.slot_id)

    print(f"\nFULL: {planner.occupied_count}/9 boxes loaded")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Standalone 3x3 outbound AMR load planner"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--demo", action="store_true", help="run a 9-box example")
    mode.add_argument("--test", action="store_true", help="run built-in unit tests")
    args = parser.parse_args()

    if args.test:
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(
            OutboundLoadPlannerTests
        )
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        raise SystemExit(0 if result.wasSuccessful() else 1)

    run_demo()


if __name__ == "__main__":
    main()
