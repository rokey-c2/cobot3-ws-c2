"""Verify that a D run cannot leave OUT rendering enabled after reset."""
import ast
from pathlib import Path
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]


def load_methods(relative, class_name, names):
    path = ROOT / relative
    tree = ast.parse(path.read_text())
    original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    selected = ast.ClassDef(name=class_name, bases=[], keywords=[], decorator_list=[],
                           body=[n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in names])
    module = ast.fix_missing_locations(ast.Module(body=[selected], type_ignores=[]))
    namespace = {}
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace[class_name]


class OutCameraResetTest(unittest.TestCase):
    def test_reset_suspends_rendering_and_next_d_reuses_camera(self):
        camera_class = load_methods('isaac_sim/robots/p3020/vision/camera.py', 'CameraInterface', {'set_enabled', 'post_reset'})
        camera = camera_class()
        camera._camera = Mock()
        camera.initialize = Mock()
        agent_class = load_methods('isaac_sim/robots/p3020/p3020_out_mission_agent.py', 'P3020UnloadToBinAgent', {'reset_camera', 'enable_camera'})
        agent = agent_class()
        agent.camera = camera
        agent._camera_initialized = False
        agent.camera_enabled = False
        updates = camera._camera._render_product.hydra_texture.set_updates_enabled

        agent.enable_camera()
        camera.initialize.assert_called_once()
        updates.assert_called_with(True)
        agent.reset_camera()
        self.assertFalse(agent.camera_enabled)
        updates.assert_called_with(False)
        camera._camera.pause.assert_called_once()
        agent.enable_camera()
        updates.assert_called_with(True)
        camera.initialize.assert_called_once()
        self.assertTrue(agent.camera_enabled)

    def test_reset_before_first_d_does_not_create_camera(self):
        cls = load_methods('isaac_sim/robots/p3020/p3020_out_mission_agent.py', 'P3020UnloadToBinAgent', {'reset_camera'})
        agent = cls()
        agent.camera = Mock()
        agent._camera_initialized = False
        agent.camera_enabled = False
        agent.reset_camera()
        agent.camera.post_reset.assert_not_called()
        agent.camera.set_enabled.assert_not_called()


if __name__ == '__main__':
    unittest.main()
