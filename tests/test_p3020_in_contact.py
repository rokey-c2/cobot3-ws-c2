"""Regressions for IN pickup pressure and the DESCEND state-boundary jerk."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from test_p3020_outcomes import ROOT, load_class


class InContactFsmTest(unittest.TestCase):
    def setUp(self):
        cls=load_class(ROOT/'isaac_sim/robots/p3020/p3020_mission_agent.py',
            'PickPlaceFSM', {'current_action','lift_after_contact','_build_waypoints'}, {
                'np':np, 'ArticulationAction':SimpleNamespace,
                'APPROACH_HEIGHT_OFFSET':.35, 'MIN_TRANSIT_Z_FOR_POD':.86,
            })
        self.fsm=cls()

    def test_transition_holds_actual_joints_instead_of_replaying_approach(self):
        f=self.fsm
        f.start=None
        f.step=0
        f.mode='joint'
        f.joint_start=np.array([1.,2.])
        f._robot=Mock(dof_names=['a','b'])
        f._robot.get_joint_positions.return_value=np.array([.1,.2])
        action,solved=f.current_action(None)
        self.assertTrue(solved)
        np.testing.assert_array_equal(action.joint_positions,[.1,.2])

    def test_contact_cancels_descent_and_preserves_placement_clearance(self):
        f=self.fsm
        for state in (1,2):
            f.state=state;f.step=70;f.start=np.array([0,0,.9])
            f.pick_xy=np.array([1.65,-1.32]);f.place_xy=np.array([-.5,0.])
            f.pick_z=.6368;f.place_z=1.17
            held_offset=.204
            f.lift_after_contact(place_z=.8+.175+held_offset+.01)
            self.assertEqual(f.state,3)
            self.assertEqual(f.gripper,'close')
            self.assertIsNone(f.start)
            self.assertEqual(f.step,0)
            self.assertAlmostEqual(f.waypoints[5][2]-held_offset-.175,.81)
            self.assertGreater(f.waypoints[3][2],f.pick_z)


class Transform:
    def __init__(self,matrix):self.matrix=np.asarray(matrix)
    def ExtractTranslation(self):return self.matrix[:3,3].copy()
    def Transform(self,point):return (self.matrix@np.r_[point,1.])[:3]
    def GetInverse(self):return Transform(np.linalg.inv(self.matrix))


class GripperOffsetTest(unittest.TestCase):
    def make_gripper(self,preserve):
        self.body=Mock()
        self.grip=np.eye(4);self.grip[:3,3]=[1.65,-1.32,.6368]
        self.box=np.eye(4);self.box[:3,3]=[1.65,-1.32,.4518]
        stage=SimpleNamespace(GetPrimAtPath=lambda path:path)
        def xform(prim):
            return SimpleNamespace(ComputeLocalToWorldTransform=lambda t:Transform(self.grip if prim=='grip' else self.box))
        cls=load_class(ROOT/'isaac_sim/robots/p3020/contact_gripper.py','ContactGripper',
            {'_attach','_local_snap_target','update','detach'}, {
                'Usd':SimpleNamespace(TimeCode=SimpleNamespace(Default=lambda:0)),
                'UsdGeom':SimpleNamespace(Xformable=xform),
                'UsdPhysics':SimpleNamespace(RigidBodyAPI=lambda prim:self.body),
                'Gf':SimpleNamespace(Vec3d=lambda *v:np.asarray(v)),
            })
        g=cls();g._stage=stage;g._gripper_body_path='grip'
        g._preserve_contact_pose=preserve;g._contact_offset=None;g._attached_to=None
        g._local_pos=[0,0,0];g._local_down_dir=np.array([0.,0.,-1.]);g._snap_distance=.185
        g._set_object_translate=lambda path,p:self.box.__setitem__((slice(0,3),3),p)
        return g

    def test_in_attachment_keeps_actual_parcel_pose_then_lifts(self):
        g=self.make_gripper(True)
        # Reproduce a cup already 24 mm below the intended pickup height.
        # The contact guard prevents that in IN; offset preservation also must
        # never add a second downward teleport if depth/pose is imperfect.
        self.grip[2,3]-=.024
        before=self.box[:3,3].copy()
        g._attach('box')
        np.testing.assert_allclose(self.box[:3,3],before)
        self.grip[2,3]+=.10
        g.update()
        np.testing.assert_allclose(self.box[:3,3],before+[0,0,.10])
        g.detach()
        self.assertIsNone(g._contact_offset)
        self.body.CreateKinematicEnabledAttr.return_value.Set.assert_called_with(False)

    def test_default_out_attachment_still_uses_existing_snap_offset(self):
        g=self.make_gripper(False)
        self.grip[2,3]=1.
        g._attach('box')
        self.assertAlmostEqual(self.box[2,3],.815)
