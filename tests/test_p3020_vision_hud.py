"""OUT frame matching and live HUD regressions without the Isaac GPU runtime."""
import json
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from test_p3020_outcomes import ROOT, load_class


class OutVisionTest(unittest.TestCase):
    def setUp(self):
        self.clock = 0.0
        self.convert = Mock(return_value=np.array([-15.4, -1.6, 1.1]))
        cls = load_class(
            ROOT / 'isaac_sim/robots/p3020/p3020_out_mission_agent.py',
            'P3020UnloadToBinAgent', {'_wait_for_detection'}, {
                'np': np, 'time': SimpleNamespace(monotonic=lambda: self.clock),
                'rclpy': Mock(), 'pixel_to_world_xy': self.convert,
                'is_within_reach': lambda xy: xy[0] > -16,
            },
        )
        self.agent = cls()
        self.frame = np.ones((4, 4, 4), dtype=np.uint8)
        self.depth = np.ones((4, 4))
        self.agent.camera = SimpleNamespace(
            get_frame=lambda: self.frame, get_depth=lambda: self.depth,
        )
        self.agent.world = Mock()
        self.agent.world.step.side_effect = lambda **kw: setattr(self, 'clock', self.clock + .5)
        self.bridge = Mock()
        self.bridge.publish_image.return_value = 123
        self.box = {'cx': 2., 'cy': 2., 'w': 2., 'h': 2., 'conf': .968}
        self.bridge.take_detection_result.return_value = {'candidates': [self.box]}

    def scan(self, steps=5):
        return self.agent._wait_for_detection(self.bridge, steps, None, 1/60)

    def test_accepted_box_publishes_size_and_confidence_for_hud(self):
        np.testing.assert_allclose(self.scan(), [-15.4, -1.6, 1.1])
        self.bridge.publish_validated_detection.assert_called_with(123, self.box)

    def test_late_result_uses_saved_rgb_depth_and_retries_same_stamp(self):
        def result(stamp):
            if self.clock < 1.:
                self.frame[:] = 99
                self.depth[:] = 99
                return None
            return {'candidates': [self.box]}
        self.bridge.take_detection_result.side_effect = result
        self.scan()
        np.testing.assert_equal(self.convert.call_args.args[1], np.ones((4, 4)))
        np.testing.assert_equal(self.convert.call_args.args[3], np.ones((4, 4, 4)))
        self.assertEqual(self.bridge.publish_image.call_args.kwargs, {'stamp_ns': 123})

    def test_rejected_or_empty_result_never_locks_hud(self):
        for world in (None, np.array([-17., -1., 1.])):
            self.convert.return_value = world
            self.bridge.publish_validated_detection.reset_mock()
            self.assertIsNone(self.scan())
            self.assertTrue(all(call.args[1] is None for call in self.bridge.publish_validated_detection.call_args_list))
        self.bridge.take_detection_result.return_value = {'candidates': []}
        self.assertIsNone(self.scan())
        self.assertIsNone(self.bridge.publish_validated_detection.call_args.args[1])

    def test_missing_depth_does_not_issue_inference(self):
        self.depth = None
        self.assertIsNone(self.scan())
        self.bridge.publish_image.assert_not_called()


class HudStatusTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        cls = load_class(
            ROOT / 'isaac_sim/robots/p3020/vision/box_detector_node.py',
            'BoxDetectorNode', {'_on_validated_detection', 'stream_health'}, {
                'np': np, 'json': json, 'String': object,
                'time': SimpleNamespace(monotonic=lambda: self.now),
            },
        )
        self.node = cls()
        self.node._detection_lock = threading.Lock()
        self.node._latest_detection = None
        self.node._latest_detection_time = 0.
        self.node._latest_source = (object(), np.zeros((4, 4, 3)))
        self.node._latest_source_time = self.now
        self.node.has_stream_frame = True
        self.node.stream_path = '/stream.mjpg'
        self.node.draw_laser_hud = Mock()
        self.node._update_web_stream = Mock()
        self.node._publish_annotated = Mock()
        self.node.get_logger = Mock()
        self.box = {'cx': 2., 'cy': 2., 'w': 2., 'h': 2., 'conf': .968}

    def send(self, detection):
        self.node._on_validated_detection(SimpleNamespace(data=json.dumps({'detection': detection})))

    def test_validation_updates_stream_without_waiting_for_another_frame(self):
        self.send(self.box)
        self.node._update_web_stream.assert_called_once()
        self.assertEqual(self.node.stream_health()['detection_state'], 'TARGET LOCKED')
        self.assertEqual(self.node.stream_health()['confidence'], .968)
        self.send(None)
        self.assertEqual(self.node.stream_health()['detection_state'], 'SCANNING')

    def test_expired_detection_and_frozen_video_do_not_report_locked(self):
        self.send(self.box)
        self.now += 1.1
        self.assertEqual(self.node.stream_health()['detection_state'], 'SCANNING')
        self.now += 3.
        self.assertEqual(self.node.stream_health()['detection_state'], 'WAITING')
        self.assertFalse(self.node.stream_health()['live'])


if __name__ == '__main__':
    unittest.main()
