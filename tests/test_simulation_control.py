"""Physical joint targets, effort bounds, contact and reset tests."""

import importlib.util
import struct
import unittest

from robot_arm_library.physics.control import JointServos


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is not installed")
class JointServoTests(unittest.TestCase):
    def setup_model(self, xml, names, **options):
        import mujoco
        model = mujoco.MjModel.from_xml_string(xml.replace("<mujoco>", '<mujoco><compiler angle="radian"/>', 1))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        manifest = {"robots": [{"prefix": "r_", "qpos": {name: 0 for name in names}}]}
        return mujoco, model, data, JointServos(mujoco, model, data, manifest, **options)

    def test_servo_moves_without_teleporting_and_respects_native_effort(self):
        mujoco, model, data, servo = self.setup_model(
            '<mujoco><option gravity="0 0 0"/><worldbody><body><joint name="r_hinge" range="-1 1" actuatorfrcrange="-.9 .9"/>'
            '<geom type="capsule" fromto="0 0 0 .2 0 0" size=".02" mass="1"/></body></worldbody>'
            '<actuator><position joint="r_hinge" kp="100" forcerange="-.4 .4" gear="2"/></actuator></mujoco>', ["hinge"])
        original = model.actuator_gainprm.copy()
        servo.set_targets({"r_hinge": .7})
        self.assertEqual(float(data.qpos[0]), 0)
        self.assertEqual(servo.metadata["r_hinge"]["effort_range"], [-.8, .8])
        for _ in range(1500):
            servo.step()
            self.assertLessEqual(abs(float(data.qfrc_applied[0])), .8)
            mujoco.mj_step(model, data)
        self.assertAlmostEqual(float(data.qpos[0]), .7, delta=.015)
        self.assertAlmostEqual(float(data.qfrc_actuator[0]), 0, delta=1e-8)
        servo.reset()
        self.assertEqual(servo.targets, {})
        self.assertTrue((model.actuator_gainprm == original).all())
        self.assertEqual(float(data.qfrc_applied[0]), 0)
        with self.assertRaises(ValueError):
            servo.set_targets({"r_hinge": 2})
        with self.assertRaises(ValueError):
            servo.set_targets({"r_hinge": float("nan")})

    def test_float32_slider_endpoints_clamp_to_exact_joint_limits(self):
        _, _, data, servo = self.setup_model(
            '<mujoco><worldbody><body><joint name="r_elbow-flex" range="-1.69 1.69"/>'
            '<geom size=".1"/></body></worldbody></mujoco>', ["elbow-flex"])
        for limit in (-1.69, 1.69):
            rounded = struct.unpack("f", struct.pack("f", limit))[0]
            with self.subTest(limit=limit, rounded=rounded):
                self.assertGreater(abs(rounded), abs(limit))
                servo.set_targets({"r_elbow-flex": rounded})
                self.assertEqual(servo.targets["r_elbow-flex"], limit)
                self.assertEqual(float(data.qpos[0]), 0)
        before = dict(servo.targets)
        for target in (-1.6901, 1.6901, float("nan"), float("inf")):
            with self.subTest(target=target), self.assertRaises(ValueError):
                servo.set_targets({"r_elbow-flex": target})
            self.assertEqual(servo.targets, before)

    def test_contact_blocks_slide_target(self):
        mujoco, _, data, servo = self.setup_model(
            '<mujoco><option gravity="0 0 0"/><worldbody><geom type="box" pos=".4 0 .2" size=".1 .2 .2"/>'
            '<body pos="0 0 .2"><joint name="r_slide" type="slide" axis="1 0 0" range="-1 1"/>'
            '<geom type="box" size=".1 .1 .1" mass="1"/></body></worldbody></mujoco>', ["slide"], max_effort=5)
        servo.set_targets({"r_slide": .8})
        self.assertEqual(float(data.qpos[0]), 0)
        for _ in range(1500):
            servo.step()
            mujoco.mj_step(servo.model, data)
        self.assertGreater(data.ncon, 0)
        self.assertGreater(float(data.qpos[0]), .15)
        self.assertLess(float(data.qpos[0]), .25)
        self.assertLessEqual(abs(float(data.qfrc_applied[0])), 5)

    def test_fixed_tendon_coupled_joints_not_silently_overridden(self):
        _, model, _, servo = self.setup_model(
            '<mujoco><worldbody><body><joint name="r_a"/><geom size=".1"/></body>'
            '<body pos="1 0 0"><joint name="r_b"/><geom size=".1"/></body></worldbody>'
            '<tendon><fixed name="r_t"><joint joint="r_a" coef="1"/><joint joint="r_b" coef="1"/></fixed></tendon>'
            '<actuator><motor name="r_motor" tendon="r_t"/></actuator></mujoco>', ["a", "b"])
        original = model.actuator_gainprm.copy()
        self.assertFalse(servo.metadata["r_a"]["available"])
        with self.assertRaises(ValueError):
            servo.set_targets({"r_a": .2})
        self.assertTrue((model.actuator_gainprm == original).all())
