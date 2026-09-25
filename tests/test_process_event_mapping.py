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
                "INPUT_ZONE", "AMR_PICKUP", "AMR_NAVIGATION",
                "MANIPULATOR_PICK", "MANIPULATOR_PLACE", "MAIN_CONVEYOR",
                "SORTER_A", "SORTER_B", "REGION_B", "COMPLETE",
            ],
        )

    def test_p3020_success_advances_package_to_main_conveyor(self):
        result = self.module.interpret_process_event(
            {"event_type": "P3020_STATE", "state": "DONE_SUCCESS"}
        )
        self.assertEqual(result["stage_code"], "MAIN_CONVEYOR")
        self.assertEqual(result["zone_code"], "MAIN_CONVEYOR")
        self.assertFalse(result["failed"])

    def test_sorter_arrival_completes_mission_in_destination_zone(self):
        result = self.module.interpret_process_event(
            {"event_type": "SORTER_STATE", "state": "ARRIVED:C"}
        )
        self.assertEqual(result["stage_code"], "COMPLETE")
        self.assertEqual(result["zone_code"], "REGION_C")
        self.assertTrue(result["complete"])

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
