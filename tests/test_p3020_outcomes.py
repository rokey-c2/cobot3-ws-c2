"""Exercise production control methods without importing Isaac's GPU runtime."""
import ast
import queue
import threading
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
AGENT = ROOT / "isaac_sim/robots/p3020/p3020_mission_agent.py"
ACTION = ROOT / "ros2_ws/src/arm_controller/arm_controller/pick_place_action_server.py"


def load_class(path, name, methods, namespace):
    tree = ast.parse(path.read_text())
    original = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == name)
    selected = ast.ClassDef(name=name, bases=[], keywords=[], decorator_list=[], body=[
        node for node in original.body if isinstance(node, ast.FunctionDef) and node.name in methods
    ])
    module = ast.fix_missing_locations(ast.Module(body=[selected], type_ignores=[]))
    exec(compile(module, str(path), "exec"), namespace)
    return namespace[name]


class VisionError(RuntimeError):
    pass


class PixelHeightFilterTest(unittest.TestCase):
    def test_rejects_reproduced_low_background_height_and_keeps_box_height(self):
        tree = ast.parse(AGENT.read_text())
        function = next(
            node for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "pixel_to_world_xy"
        )
        module = ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[]))
        namespace = {
            "np": np,
            "MIN_VALID_SCAN_DEPTH": 0.4,
            "MIN_BOX_WORLD_Z": 0.40,
            "KNOWN_LOW_FALSE_POSITIVE_XY": np.array([2.265, -1.396]),
            "KNOWN_LOW_FALSE_POSITIVE_RADIUS": 0.20,
            "_looks_like_box_color": lambda *args: True,
        }
        exec(compile(module, str(AGENT), "exec"), namespace)
        convert = namespace["pixel_to_world_xy"]
        depth = np.ones((4, 4), dtype=float)
        camera = SimpleNamespace(
            pixel_to_world=lambda cx, cy, distance: np.array([2.265, -1.396, self.z])
        )

        frame = np.full((4, 4, 3), 100)
        for self.z in (0.312, 0.589):
            with self.subTest(z=self.z):
                self.assertIsNone(convert((1, 1, .95), depth, camera, frame))

        # The same valid box height remains detectable outside the known false-positive area.
        camera.pixel_to_world = lambda cx, cy, distance: np.array([1.8, -1.2, 0.589])
        np.testing.assert_allclose(
            convert((1, 1, .95), depth, camera, frame), [1.8, -1.2, 0.589]
        )


class BatchOutcomeTest(unittest.TestCase):
    def setUp(self):
        cls = load_class(AGENT, "P3020PickPlaceAgent", {"run_until_cargo_empty"}, {
            "VisionError": VisionError, "MAX_PICK_ATTEMPTS": 3, "NO_BOX_CONFIRM_TIMEOUT_S": 5,
        })
        self.agent = cls()
        self.bridge = Mock()
        self.amr = Mock()

    def run_batch(self, outcomes, confirm=None):
        self.agent.run_pick_place = Mock(side_effect=outcomes)
        self.agent._wait_for_detection = Mock(return_value=confirm)
        return self.agent.run_until_cargo_empty(self.bridge, (0, 0), self.amr)

    def statuses(self):
        return [call.args[0] for call in self.bridge.publish_status.call_args_list]

    def test_healthy_empty_is_not_failure_or_direct_amr_command(self):
        self.assertTrue(self.run_batch([(False, "NO_BOX")]))
        self.assertEqual(self.statuses(), ["CHECKING_EMPTY", "CARGO_EMPTY"])
        self.amr.request_return_dock.assert_not_called()

    def test_grasp_error_then_no_detection_cannot_become_success(self):
        self.assertFalse(self.run_batch([(False, "grasp failed"), (False, "NO_BOX"), (False, "NO_BOX")]))
        self.assertEqual(self.statuses()[-1], "DONE_FAIL:grasp failed")
        self.agent._wait_for_detection.assert_not_called()
        self.assertEqual(self.agent.run_pick_place.call_count, 3)

    def test_successful_retry_clears_error_before_empty_check(self):
        self.assertTrue(self.run_batch([(False, "unreachable"), (True, "placed"), (False, "NO_BOX")]))
        self.assertFalse(any(s.startswith("DONE_FAIL") for s in self.statuses()))

    def test_lost_detector_is_fatal(self):
        self.assertFalse(self.run_batch([VisionError("no response")]))
        self.assertEqual(self.statuses(), ["DONE_FAIL:VISION_ERROR:no response"])

    def test_lost_detector_during_empty_confirmation_is_fatal(self):
        self.agent.run_pick_place = Mock(return_value=(False, "NO_BOX"))
        self.agent._wait_for_detection = Mock(side_effect=VisionError("frozen camera"))
        self.assertFalse(self.agent.run_until_cargo_empty(self.bridge, (0, 0), self.amr))
        self.assertEqual(self.statuses()[-1], "DONE_FAIL:VISION_ERROR:frozen camera")


class DetectionTest(unittest.TestCase):
    def setUp(self):
        self.clock = 0.0
        self.frame_id = 0
        self.rgb = np.full((2, 2, 4), 100, dtype=np.uint8)
        self.depth = np.ones((2, 2))
        self.convert = Mock(return_value=np.array([1., 2., 3.]))
        cls = load_class(AGENT, "P3020PickPlaceAgent", {"_wait_for_detection"}, {
            "VisionError": VisionError, "time": SimpleNamespace(monotonic=lambda: self.clock),
            "np": np, "rclpy": SimpleNamespace(spin_once=lambda *a, **k: None),
            "VISION_RESPONSE_TIMEOUT_S": 10, "MIN_EMPTY_FRAMES": 3,
            "pixel_to_world_xy": self.convert,
        })
        self.agent = cls()
        self.agent.camera = SimpleNamespace(
            get_frame_id=lambda: self.frame_id, get_frame=lambda: self.rgb, get_depth=lambda: self.depth,
        )
        self.agent.world = SimpleNamespace(step=self.step)
        self.bridge = Mock()
        self.bridge.publish_image.side_effect = lambda frame, stamp_ns=None: self.frame_id if stamp_ns is None else stamp_ns
        self.bridge.take_detection_result.side_effect = lambda stamp: {"detection": None}

    def step(self, **kwargs):
        self.clock += 1
        self.frame_id += 1

    def scan(self):
        return self.agent._wait_for_detection(self.bridge, 5, None, 1)

    def test_explicit_fresh_negative_results_confirm_empty(self):
        self.assertIsNone(self.scan())
        self.assertGreaterEqual(self.bridge.publish_image.call_count, 3)
        self.assertGreaterEqual(self.clock, 5)

    def test_slow_simulation_does_not_stretch_empty_confirmation(self):
        # Each render takes one wall second but advances only 1/60 sim second.
        self.assertIsNone(self.agent._wait_for_detection(self.bridge, 300, None, 1 / 60))
        self.assertGreaterEqual(self.clock, 5)
        self.assertLessEqual(self.clock, 7)
        self.assertGreaterEqual(self.bridge.publish_image.call_count, 3)

    def test_valid_box_during_confirmation_prevents_empty(self):
        self.bridge.take_detection_result.side_effect = lambda stamp: (
            {"detection": {"cx": 1, "cy": 1, "conf": .9}}
            if self.clock >= 4 else {"detection": None}
        )
        np.testing.assert_equal(
            self.agent._wait_for_detection(self.bridge, 300, None, 1 / 60), [1, 2, 3]
        )

    def test_missing_result_never_means_empty(self):
        self.bridge.take_detection_result.return_value = None
        self.bridge.take_detection_result.side_effect = None
        with self.assertRaises(VisionError):
            self.scan()

    def test_frozen_camera_never_means_empty(self):
        self.agent.camera.get_frame_id = lambda: 1
        with self.assertRaises(VisionError):
            self.scan()

    def test_missing_depth_never_means_empty(self):
        self.depth = None
        with self.assertRaises(VisionError):
            self.scan()

    def test_rejected_false_positive_keeps_scanning(self):
        self.bridge.take_detection_result.side_effect = lambda stamp: {
            "detection": {"cx": 1, "cy": 1, "conf": .9}
        }
        self.convert.return_value = None
        self.assertIsNone(self.scan())
        self.assertGreaterEqual(self.convert.call_count, 3)

    def test_dropped_request_retries_same_frame_and_saved_depth(self):
        self.bridge.take_detection_result.side_effect = lambda stamp: (
            {"detection": {"cx": 1, "cy": 1, "conf": .9}}
            if self.bridge.publish_image.call_count >= 2 else None
        )
        np.testing.assert_equal(self.scan(), [1, 2, 3])
        calls = self.bridge.publish_image.call_args_list
        self.assertEqual(calls[1].kwargs["stamp_ns"], 1)
        np.testing.assert_equal(calls[0].args[0], calls[1].args[0])
        np.testing.assert_equal(self.convert.call_args.args[1], np.ones((2, 2)))

    def test_detection_uses_saved_depth_not_new_frame(self):
        calls = 0
        def reply(stamp):
            nonlocal calls
            calls += 1
            if calls == 1:
                self.depth = np.full((2, 2), 99.)
                return None
            return {"detection": {"cx": 1, "cy": 1, "conf": .9}}
        self.bridge.take_detection_result.side_effect = reply
        np.testing.assert_equal(self.scan(), [1, 2, 3])
        np.testing.assert_equal(self.convert.call_args.args[1], np.ones((2, 2)))


class OutParcelSelectionTest(unittest.TestCase):
    def test_detected_parcel_reaches_world_pose_target(self):
        path = ROOT / "isaac_sim/robots/p3020/p3020_out_mission_agent.py"
        transform = Mock()
        transform.ComputeLocalToWorldTransform.return_value.ExtractTranslation.return_value = (-15.4, -1.6, 1.)
        pxr = SimpleNamespace(
            Gf=Mock(), Usd=SimpleNamespace(TimeCode=SimpleNamespace(Default=lambda: 0)),
            UsdGeom=SimpleNamespace(Xformable=lambda prim: transform), UsdPhysics=Mock(),
        )
        # Honor the production imports so missing USD bindings fail on this path.
        namespace = {
            alias.asname or alias.name: getattr(pxr, alias.name)
            for node in ast.parse(path.read_text()).body
            if isinstance(node, ast.ImportFrom) and node.module == "pxr"
            for alias in node.names
        }
        namespace.update(
            np=np, IDLE_SCAN_TIMEOUT_STEPS=90, REFINE_WAIT_TIMEOUT_STEPS=100,
            SCAN_DESCEND_STEPS=150, SCAN_MID_HEIGHT=1.4, APPROACH_HEIGHT=1.2,
            PARCEL_HALF_HEIGHT=.175, MAX_VISION_PARCEL_DISTANCE=.4, find_nearest_parcel=lambda *args: "box",
            base_relative=lambda xy: np.asarray(xy) - [-15., -2.],
            is_within_reach=lambda xy: np.linalg.norm(np.asarray(xy) - [-15., -2.]) <= 2.,
            get_tcp_pose=lambda frame: [0., 0., 1.5],
        )
        cls = load_class(path, "P3020UnloadToBinAgent", {"_locate_box_and_descend"}, namespace)
        agent = cls()
        agent.stage = Mock()
        agent._placed_parcel_paths = set()
        agent.ee_frame = Mock()
        agent._wait_for_detection = Mock(side_effect=[np.array([-15.446, -1.626, 1.126]), None])
        agent._move_to = Mock()
        np.testing.assert_allclose(
            agent._locate_box_and_descend(Mock(), None, 1 / 60), [-15.4, -1.6, 1.175]
        )
        self.assertEqual(agent._move_to.call_count, 2)
        # A visually reachable candidate must not freeze an actually distant box.
        transform.ComputeLocalToWorldTransform.return_value.ExtractTranslation.return_value = (-17.05, -2., 1.)
        agent._wait_for_detection = Mock(return_value=np.array([-16.9, -2., 1.]))
        agent._move_to.reset_mock()
        pxr.UsdPhysics.reset_mock()
        self.assertIsNone(agent._locate_box_and_descend(Mock(), None, 1 / 60))
        agent._move_to.assert_not_called()
        pxr.UsdPhysics.RigidBodyAPI.assert_not_called()

        # Previously placed boxes must never start another approach.
        agent._placed_parcel_paths.add("box")
        transform.ComputeLocalToWorldTransform.return_value.ExtractTranslation.return_value = (-15.4, -1.6, 1.)
        agent._wait_for_detection = Mock(return_value=np.array([-15.4, -1.6, 1.]))
        self.assertIsNone(agent._locate_box_and_descend(Mock(), None, 1 / 60))
        agent._move_to.assert_not_called()

        cls = load_class(path, "P3020UnloadToBinAgent", {"_placement_verified"}, namespace)
        verifier = cls()
        verifier.stage = agent.stage
        verifier.gripper = Mock()
        verifier.gripper.is_attached.return_value = False
        self.assertTrue(verifier._placement_verified("box", (-15.4, -1.6, 1.)))
        self.assertFalse(verifier._placement_verified("box", (-15.4, -1.6, .5)))
        verifier.gripper.is_attached.return_value = True
        self.assertFalse(verifier._placement_verified("box", (-15.4, -1.6, 1.)))


class OutHomeReturnTest(unittest.TestCase):
    def test_return_requires_measured_home_pose(self):
        path = ROOT / "isaac_sim/robots/p3020/p3020_out_mission_agent.py"
        cls = load_class(path, "P3020UnloadToBinAgent", {"_return_to_ready_pose"}, {
            "np": np, "lerp": lambda a, b, t: a + (b - a) * t,
            "ArticulationAction": SimpleNamespace,
            "clamp_to_safe_limits": lambda action, names: action,
        })
        for reached in (True, False):
            with self.subTest(reached=reached):
                agent = cls()
                agent.home_q = np.zeros(2)
                agent._home_return_failed = False
                agent.robot = Mock(dof_names=["a", "b"])
                agent.robot.get_joint_positions.side_effect = (
                    [np.ones(2), np.zeros(2)] if reached else None
                )
                agent.robot.get_joint_positions.return_value = np.ones(2)
                agent.world = Mock()
                self.assertEqual(agent._return_to_ready_pose(steps=1), reached)
                self.assertEqual(agent._home_return_failed, not reached)


class ActionOutcomeTest(unittest.TestCase):
    def test_fatal_status_survives_later_feedback_and_aborts_action(self):
        namespace = {
            "String": SimpleNamespace, "PickPlace": SimpleNamespace(Result=SimpleNamespace, Feedback=SimpleNamespace),
            "json": __import__("json"), "RESULT_TIMEOUT_SEC": 1, "_PROGRESS_BY_STATE": {},
        }
        cls = load_class(ACTION, "PickPlaceActionServer", {"_on_status", "_execute_callback"}, namespace)
        server = cls()
        server.get_logger = Mock()
        server.command_pub = Mock()
        def statuses(_):
            server._on_status(SimpleNamespace(data="DONE_FAIL:grasp failed"))
            server._on_status(SimpleNamespace(data="SCANNING"))
        namespace["time"] = SimpleNamespace(sleep=statuses)
        pose = SimpleNamespace(position=SimpleNamespace(x=0, y=0))
        goal = Mock(request=SimpleNamespace(object_id="box", pickup_pose=pose, place_pose=pose))
        result = server._execute_callback(goal)
        self.assertFalse(result.success)
        self.assertEqual(result.message, "grasp failed")
        goal.abort.assert_called_once()
        goal.succeed.assert_not_called()

    def test_per_box_success_does_not_finish_batch(self):
        namespace = {
            "String": SimpleNamespace, "PickPlace": SimpleNamespace(Result=SimpleNamespace, Feedback=SimpleNamespace),
            "json": __import__("json"), "RESULT_TIMEOUT_SEC": 1, "_PROGRESS_BY_STATE": {},
        }
        cls = load_class(ACTION, "PickPlaceActionServer", {"_on_status", "_execute_callback"}, namespace)
        server = cls()
        server.get_logger = Mock()
        server.command_pub = Mock()
        updates = iter(["DONE_SUCCESS", "CHECKING_EMPTY", "CARGO_EMPTY"])
        namespace["time"] = SimpleNamespace(sleep=lambda _: server._on_status(SimpleNamespace(data=next(updates))))
        pose = SimpleNamespace(position=SimpleNamespace(x=0, y=0))
        goal = Mock(request=SimpleNamespace(object_id="box", pickup_pose=pose, place_pose=pose))
        result = server._execute_callback(goal)
        self.assertTrue(result.success)
        self.assertEqual(goal.publish_feedback.call_count, 2)
        goal.succeed.assert_called_once()
        goal.abort.assert_not_called()


class PlacementTest(unittest.TestCase):
    def test_missing_or_misplaced_parcel_is_not_verified(self):
        transform = Mock()
        namespace = {"np": np, "UsdGeom": SimpleNamespace(Xformable=lambda prim: transform),
                     "Usd": SimpleNamespace(TimeCode=SimpleNamespace(Default=lambda: 0)),
                     "PARCEL_HALF_HEIGHT": .175, "CONVEYOR_SURFACE_Z": .8, "PLACE_Z_TOLERANCE": .1}
        cls = load_class(AGENT, "P3020PickPlaceAgent", {"_parcel_at_place"}, namespace)
        agent = cls()
        agent.stage = Mock()
        agent.stage.GetPrimAtPath.return_value.IsValid.return_value = True
        for xyz, expected in [((0, 0, .975), True), ((.3, 0, .975), False), ((0, 0, .2), False)]:
            with self.subTest(xyz=xyz):
                transform.ComputeLocalToWorldTransform.return_value.ExtractTranslation.return_value = xyz
                self.assertEqual(agent._parcel_at_place("box", (0, 0), .15), expected)
        agent.stage.GetPrimAtPath.return_value.IsValid.return_value = False
        self.assertFalse(agent._parcel_at_place("box", (0, 0), .15))


class DetectorProtocolTest(unittest.TestCase):
    def test_preview_does_not_replace_pending_inference(self):
        path = ROOT / "isaac_sim/robots/p3020/vision/box_detector_node.py"
        cls = load_class(path, "BoxDetectorNode", {"preview_callback"}, {
            "Image": object, "time": SimpleNamespace(monotonic=lambda: 2),
        })
        detector = cls()
        detector._detection_lock = threading.Lock()
        detector._latest_detection = None
        detector._latest_detection_time = 0
        detector._inference_queue = queue.Queue(maxsize=1)
        pending = object()
        detector._inference_queue.put(pending)
        detector.imgmsg_to_rgb = Mock(return_value=np.ones((2, 2, 3)))
        detector.draw_laser_hud = Mock()
        detector._update_web_stream = Mock()
        detector._publish_annotated = Mock()
        detector.preview_callback(object())
        self.assertIs(detector._inference_queue.get_nowait(), pending)
        detector._update_web_stream.assert_called_once()

    def test_negative_result_acknowledges_exact_source_frame(self):
        path = ROOT / "isaac_sim/robots/p3020/vision/box_detector_node.py"
        tree = ast.parse(path.read_text())
        name = next(node.name for node in tree.body if isinstance(node, ast.ClassDef)
                    and any(isinstance(m, ast.FunctionDef) and m.name == "image_callback" for m in node.body))
        cls = load_class(path, name, {"image_callback", "_inference_worker"}, {
            "Image": object, "String": SimpleNamespace, "json": __import__("json"),
            "time": SimpleNamespace(monotonic=lambda: 0),
            "queue": queue, "rclpy": SimpleNamespace(ok=lambda: True),
        })
        detector = cls()
        detector.imgmsg_to_rgb = Mock(return_value=np.zeros((2, 2, 3)))
        detector.detector = Mock()
        detector.detector.detect_candidates.return_value = []
        detector.result_pub = Mock()
        detector.pixel_pub = Mock()
        detector.draw_laser_hud = Mock()
        detector._publish_annotated = Mock()
        detector._update_web_stream = Mock()
        detector._detection_lock = threading.Lock()
        detector._latest_detection = None
        detector._latest_detection_time = 0
        detector._inference_queue = queue.Queue()
        detector.image_callback(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=34))))
        detector._inference_queue.put(None)
        detector._inference_worker()
        result = __import__("json").loads(detector.result_pub.publish.call_args.args[0].data)
        self.assertEqual(result, {
            "stamp_ns": 12_000_000_034,
            "candidates": [],
            "detection": None,
        })
        detector.pixel_pub.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()
