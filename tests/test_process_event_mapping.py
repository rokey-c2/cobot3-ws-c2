import importlib.util
import sys
import types
import unittest
from unittest.mock import Mock
from pathlib import Path


def load_module():
    psycopg_module = types.ModuleType("psycopg")
    psycopg_rows_module = types.ModuleType("psycopg.rows")
    psycopg_rows_module.dict_row = object()
    app_module = types.ModuleType("app")
    database_module = types.ModuleType("app.database")
    database_module.get_db_connection = lambda: None
    sys.modules.update(
        {
            "psycopg": psycopg_module,
            "psycopg.rows": psycopg_rows_module,
            "app": app_module,
            "app.database": database_module,
        }
    )
    path = (
        Path(__file__).resolve().parents[1]
        / "backend"
        / "app"
        / "process_events.py"
    )
    spec = importlib.util.spec_from_file_location("process_events_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ProcessEventMappingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()

    def test_region_b_route_passes_sorter_a_then_sorter_b(self):
        route = self.module.build_route("B")
        self.assertEqual(
            route,
            [
                "INPUT_ZONE", "AMR_PICKUP", "AMR_NAVIGATION", "AMR_ARRIVAL",
                "MANIPULATOR_PICK", "MANIPULATOR_PLACE", "MAIN_CONVEYOR",
                "SORTER_A", "SORTER_B", "REGION_B", "COMPLETE",
            ],
        )

    def test_amr_arrival_is_separate_from_in_pick(self):
        arrival = self.module.interpret_process_event(
            {"event_type": "AMR_STATE", "state": "CONVEYOR_DOCK_DONE"}
        )
        picking = self.module.interpret_process_event(
            {"event_type": "P3020_STATE", "state": "SCANNING"}
        )
        self.assertEqual(arrival["stage_code"], "AMR_ARRIVAL")
        self.assertEqual(picking["stage_code"], "MANIPULATOR_PICK")
        self.assertFalse(arrival["complete"])

    def test_p3020_success_advances_package_to_main_conveyor(self):
        result = self.module.interpret_process_event(
            {"event_type": "P3020_STATE", "state": "DONE_SUCCESS"}
        )
        self.assertEqual(result["stage_code"], "MAIN_CONVEYOR")
        self.assertEqual(result["zone_code"], "MAIN_CONVEYOR")
        self.assertFalse(result["failed"])

    def test_sorter_arrival_completes_destination_before_mission(self):
        result = self.module.interpret_process_event(
            {"event_type": "SORTER_STATE", "state": "ARRIVED:C"}
        )
        self.assertEqual(result["stage_code"], "REGION_C")
        self.assertEqual(result["zone_code"], "REGION_C")
        self.assertTrue(result["stage_completed"])
        self.assertFalse(result["complete"])

    def test_d_route_and_explicit_milestones(self):
        self.assertEqual(self.module.normalize_region("D"), "D")
        self.assertEqual(self.module.build_route("D")[-5:], ["SORTER_A", "SORTER_B", "SORTER_C", "EXCEPTION", "COMPLETE"])
        cases = [
            ("P3020_STATE", "PICK_CONFIRMED", "MANIPULATOR_PICK", True, False),
            ("P3020_STATE", "MOVING", "MANIPULATOR_PLACE", False, False),
            ("SORTER_STATE", "ENTERED:A", "SORTER_A", False, False),
            ("SORTER_STATE", "PASSED:C", "SORTER_C", True, False),
            ("P3020_OUT_STATE", "BIN_PLACED", "EXCEPTION", True, False),
            ("P3020_OUT_STATE", "DONE_SUCCESS", "COMPLETE", True, True),
        ]
        for kind, state, code, done, complete in cases:
            with self.subTest(state=state):
                result = self.module.interpret_process_event({"event_type": kind, "state": state, "region": "D"})
                self.assertEqual((result["stage_code"], result["stage_completed"], result["complete"]), (code, done, complete))

    def test_rescan_and_return_error_cannot_rewind_conveyor(self):
        stages = [{"stage_code": "MAIN_CONVEYOR", "sequence_no": 7, "status": "RUNNING"}]
        for kind, state in [("P3020_STATE", "SCANNING"), ("AMR_STATE", "ERROR")]:
            payload = {"event_type": kind, "state": state}
            self.assertEqual(self.module.progress_ignore_reason(payload, self.module.interpret_process_event(payload), stages), "package already handed to conveyor")

    def test_final_success_requires_every_physical_milestone(self):
        stages = [{"stage_code": code, "sequence_no": i, "status": "COMPLETED"}
                  for i, code in enumerate(self.module.build_route("D"), 1)]
        stages[-1]["status"] = "WAITING"
        payload = {"event_type": "P3020_OUT_STATE", "state": "DONE_SUCCESS"}
        event = self.module.interpret_process_event(payload)
        self.assertIsNone(self.module.progress_ignore_reason(payload, event, stages))
        stages[4]["status"] = "RUNNING"
        self.assertEqual(self.module.progress_ignore_reason(payload, event, stages), "physical completion milestones missing")

    def test_p3020_failure_marks_event_failed(self):
        result = self.module.interpret_process_event(
            {"event_type": "P3020_STATE", "state": "DONE_FAIL:camera timeout"}
        )
        self.assertTrue(result["failed"])
        self.assertEqual(result["zone_code"], "P3020_IN")

    def test_empty_check_and_retry_are_not_failure_or_completion(self):
        for state in ("CHECKING_EMPTY", "CARGO_EMPTY", "RETRYING:1/3:grasp failed"):
            with self.subTest(state=state):
                result = self.module.interpret_process_event({"event_type": "P3020_STATE", "state": state})
                self.assertFalse(result["failed"])
                self.assertFalse(result["complete"])
                self.assertIsNone(result["zone_code"])

    def test_batch_status_does_not_invent_package_after_completion(self):
        for state in ("CHECKING_EMPTY", "CARGO_EMPTY", "RETRYING:1/3:grasp failed"):
            cursor = Mock()
            cursor.fetchone.return_value = None
            result = self.module._find_or_create_active_tracking(
                cursor, {"event_type": "P3020_STATE", "state": state}
            )
            self.assertIsNone(result)
            self.assertEqual(cursor.execute.call_count, 1)


if __name__ == "__main__":
    unittest.main()
