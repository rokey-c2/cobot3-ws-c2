"""Exercise the production sorter geometry without starting Isaac Sim."""
import math
import unittest
from unittest.mock import Mock

from test_p3020_outcomes import ROOT, load_class


class SorterMilestonesTest(unittest.TestCase):
    def events_for(self, box_id, enabled=True):
        cls = load_class(
            ROOT / "isaac_sim/equipment/wheel_sorter/wheel_sorter_controller.py",
            "WheelSorterController", {"update_boxes", "take_process_events"}, {"math": math},
        )
        agent = cls()
        agent.REGION_TO_TRACK = {"A": "01", "B": "02", "C": "03"}
        agent.units = {track: Mock() for track in ("01", "02", "03")}
        for i, unit in enumerate(agent.units.values()):
            unit.get_world_xy.return_value = (i * 2., 0.)
        agent.enabled = {track: enabled for track in agent.units}
        agent.approach_threshold = .45
        agent.reset_threshold = .7
        agent._triggered_pairs = set()
        agent._completed_pairs = set()
        agent._diverted_pairs = set()
        agent._process_events = []
        agent._destination_d_detected = False
        agent._read_box_id = lambda path: box_id
        for x in [-.4, .8, 1.6, 2.8, 3.6, 4.8]:
            agent._box_world_xy = lambda path: (x, 0.)
            agent.update_boxes(["/World/Cargo/Parcels/box"], {1: "01", 4: None})
        events = agent.take_process_events()
        self.assertEqual(agent.take_process_events(), [])
        return events

    def test_d_emits_every_entry_and_exit_without_false_destination_arrival(self):
        events = self.events_for(4)
        self.assertEqual([e["state"] for e in events], [
            "ENTERED:A", "PASSED:A", "ENTERED:B", "PASSED:B", "ENTERED:C", "PASSED:C",
        ])
        self.assertTrue(all(e["region"] == "D" and e["package_code"] == "box" for e in events))

    def test_diversion_exit_reports_destination_before_completion(self):
        states = [e["state"] for e in self.events_for(1)]
        self.assertEqual(states[:5], ["ENTERED:A", "ROUTING:A", "PASSED:A", "ARRIVED:A", "SORTING_COMPLETE"])

    def test_disabled_sorter_never_claims_destination_success(self):
        states = [e["state"] for e in self.events_for(1, enabled=False)]
        self.assertNotIn("ARRIVED:A", states)
        self.assertNotIn("SORTING_COMPLETE", states)
