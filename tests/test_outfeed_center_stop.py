"""Verify centering/hold behavior separately from the simulator's physics test."""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from test_p3020_outcomes import ROOT, load_class


class OutfeedCenterStopTest(unittest.TestCase):
    def setUp(self):
        self.body=Mock()
        cls=load_class(ROOT/'isaac_sim/equipment/wheel_sorter/outfeed_center_stop.py',
            'OutfeedCenterStop', {'update_boxes','is_ready'}, {
                'math':math, 'Gf':SimpleNamespace(Vec3f=lambda *v:v, Vec3d=lambda *v:v),
                'UsdPhysics':SimpleNamespace(RigidBodyAPI=lambda prim:self.body),
            })
        self.controller=cls()
        c=self.controller
        c.CAPTURE_RADIUS=.7
        c.CENTER_TOLERANCE=.04
        c.MAX_SPEED=.3
        c.MIN_SPEED=.06
        c.RELEASE_DISTANCE=.2
        c.center_xy=(0.,0.)
        c.held_path=None
        c._held_z=None
        c._served=set()
        c.stage=Mock()
        c.stage.GetPrimAtPath.return_value.IsValid.return_value=True
        c.stage.GetPrimAtPath.return_value.GetAttribute.return_value.Get.return_value=4
        c._world_transform=Mock()
        c._world_transform.ExtractTranslation.return_value=(0.,0.,.768)
        c._inverse_transform=Mock()
        c._inverse_transform.TransformDir.side_effect=lambda v:v
        c._set_motion=Mock()
        c._position=Mock(return_value=(0.,.5,.975))

    def test_approach_steers_and_slows_without_holding_early(self):
        c=self.controller
        c.update_boxes(['box'])
        c._set_motion.assert_called_with((0.,-1.,0.),.3)
        self.assertFalse(c.is_ready('box'))
        self.body.CreateKinematicEnabledAttr.assert_not_called()
        c._position.return_value=(0.,.1,.975)
        c.update_boxes(['box'])
        self.assertAlmostEqual(c._set_motion.call_args.args[1],.12)

    def test_center_stops_and_holds_without_changing_position(self):
        c=self.controller
        c._position.return_value=(.02,.02,.975)
        c.update_boxes(['box'])
        self.assertTrue(c.is_ready('box'))
        c._set_motion.assert_called_with((1.,0.,0.),0.)
        self.body.CreateKinematicEnabledAttr.return_value.Set.assert_called_once_with(True)
        c.update_boxes(['box'])
        self.body.CreateKinematicEnabledAttr.return_value.Set.assert_called_once()

    def test_lift_releases_station_and_never_recaptures_placed_box(self):
        c=self.controller
        c._position.return_value=(0.,0.,.975)
        c.update_boxes(['box'])
        c._position.return_value=(0.,0.,1.2)
        c.update_boxes(['box'])
        self.assertIsNone(c.held_path)
        self.assertIn('box',c._served)
        c._position.return_value=(0.,0.,.975)
        c.update_boxes(['box'])
        self.assertFalse(c.is_ready('box'))

    def test_disabled_distant_or_non_d_parcels_do_not_start_wheels(self):
        c=self.controller
        c.update_boxes(['box'],enabled=False)
        c._set_motion.assert_called_with((1.,0.,0.),0.)
        c._position.return_value=(0.,1.,.975)
        c.update_boxes(['box'])
        c._set_motion.assert_called_with((1.,0.,0.),0.)
        c._position.return_value=(0.,.3,.975)
        c.stage.GetPrimAtPath.return_value.GetAttribute.return_value.Get.return_value=1
        c.update_boxes(['box'])
        c._set_motion.assert_called_with((1.,0.,0.),0.)
        self.assertIsNone(c.held_path)
