"""Synthetic vision, control, and mocked GPIO tests; no robot required."""
import copy
from pathlib import Path
import time
import unittest

import cv2
import numpy as np

from line_follower import (Follower, LineDetector, MotorOutput, Observation,
                           load_config, speed_to_duty, validate_config, wheel_speeds)


def config():
    return load_config(Path(__file__).with_name("config.json"))


def track_image(kind="straight", white=False):
    im = np.full((480, 640, 3), 20 if white else 230, dtype=np.uint8)
    ys = np.arange(150, 480)
    xs = np.full_like(ys, 320)
    if kind in ("right", "left"):
        offset = (120*((480-ys)/330)**2).astype(int)
        xs = xs + offset*(1 if kind == "right" else -1)
    if kind == "offset":
        xs += 90
    pts = np.column_stack((xs, ys)).astype(np.int32)
    cv2.polylines(im, [pts], False, (230,)*3 if white else (20,)*3, 20)
    return im


def obs(error=0.0, bend=0.0):
    return Observation(valid=True, error=error, bend=bend, confidence=1, reason="line")


def acquire(follower, error=0.0, start=0.0):
    for i in range(4):
        cmd = follower.step(obs(error), start+i/30, 0.01, i)
    return cmd


class VisionTests(unittest.TestCase):
    def test_straight_centered(self):
        o, _ = LineDetector(config()["vision"]).detect(track_image())
        self.assertTrue(o.valid)
        self.assertAlmostEqual(o.error, 0, delta=0.02)

    def test_curve_anticipation_and_mirror(self):
        right, _ = LineDetector(config()["vision"]).detect(track_image("right"))
        left, _ = LineDetector(config()["vision"]).detect(track_image("left"))
        self.assertTrue(right.valid and left.valid)
        self.assertGreater(right.error, 0.1)
        self.assertLess(left.error, -0.1)
        self.assertAlmostEqual(right.error, -left.error, delta=0.02)
        self.assertGreater(right.bend, 0.1)

    def test_shifted_line(self):
        o, _ = LineDetector(config()["vision"]).detect(track_image("offset"))
        self.assertTrue(o.valid)
        self.assertGreater(o.error, 0.2)

    def test_uniform_floor_or_full_black_is_not_a_line(self):
        for gray in (0, 120, 255):
            o, _ = LineDetector(config()["vision"]).detect(np.full((480,640,3), gray, np.uint8))
            self.assertFalse(o.valid)

    def test_white_tape(self):
        v = config()["vision"]
        v["line_color"] = "white"
        o, _ = LineDetector(v).detect(track_image(white=True))
        self.assertTrue(o.valid)

    def test_all_threshold_modes(self):
        for mode in ("otsu", "fixed", "adaptive"):
            v = config()["vision"]
            v["threshold_mode"] = mode
            o, _ = LineDetector(v).detect(track_image("right"))
            self.assertTrue(o.valid, mode)

    def test_shadow_gradient_adaptive(self):
        im = track_image()
        im = np.clip(im.astype(float)*np.linspace(0.45,1,640)[None,:,None], 0, 255).astype(np.uint8)
        v = config()["vision"]
        v["threshold_mode"] = "adaptive"
        o, _ = LineDetector(v).detect(im)
        self.assertTrue(o.valid)

    def test_crossing_stops(self):
        im = track_image()
        cv2.rectangle(im, (20, 310), (620, 330), (20,20,20), -1)
        o, _ = LineDetector(config()["vision"]).detect(im)
        self.assertTrue(o.ambiguous)

    def test_parallel_branches_ambiguous(self):
        im = track_image()
        cv2.line(im, (450,150), (450,479), (20,20,20), 20)
        o, _ = LineDetector(config()["vision"]).detect(im)
        self.assertTrue(o.ambiguous)

    def test_far_only_line_rejected(self):
        im = track_image()
        im[300:] = 230
        o, _ = LineDetector(config()["vision"]).detect(im)
        self.assertFalse(o.valid)


class ControllerTests(unittest.TestCase):
    def test_acquisition_requires_distinct_frames(self):
        f = Follower(config())
        for _ in range(20):
            cmd = f.step(obs(), 0.1, 0.01, 1)
        self.assertEqual(cmd.state, "WAIT_LINE")
        self.assertEqual(cmd.left, 0)

    def test_correct_turn_sign_and_forward_only(self):
        for sign in (-1, 1):
            f = Follower(config())
            cmd = acquire(f, sign*0.5)
            self.assertEqual(cmd.state, "TRACK")
            self.assertGreater(sign*(cmd.left-cmd.right), 0)
            self.assertGreaterEqual(min(cmd.left,cmd.right), 0)

    def test_curve_reduces_speed(self):
        straight, curve = Follower(config()), Follower(config())
        acquire(straight)
        acquire(curve, 0.4)
        for i in range(4, 30):
            a = straight.step(obs(), i/30, .01, i)
            b = curve.step(obs(.4, .5), i/30, .01, i)
        self.assertLess(b.left+b.right, a.left+a.right)

    def test_loss_stops_immediately_then_searches(self):
        f = Follower(config())
        acquire(f, .4)
        a = f.step(Observation(), .2, .01, 4)
        self.assertEqual((a.left,a.right),(0,0))
        self.assertEqual(a.state,"BRAKE")
        b = f.step(Observation(), .5, .01, 5)
        self.assertEqual(b.state,"SEARCH")
        self.assertGreater(b.left,0)
        self.assertLess(b.right,0)

    def test_search_sweeps_other_side(self):
        f = Follower(config())
        acquire(f, .4)
        f.step(Observation(), .2, .01, 4)
        for i in range(5, 60):
            cmd = f.step(Observation(), .2+(i-4)*.04, .01, i)
        self.assertEqual(cmd.state, "SEARCH")
        self.assertLess(cmd.left, 0)
        self.assertGreater(cmd.right, 0)

    def test_loss_without_direction_stops(self):
        f = Follower(config())
        acquire(f)
        f.step(Observation(), .2, .01, 4)
        a = f.step(Observation(), .5, .01, 5)
        self.assertEqual(a.state,"STOPPED")

    def test_recovery_disabled(self):
        cfg = config()
        cfg["recovery"]["enabled"] = False
        f = Follower(cfg)
        acquire(f, .4)
        f.step(Observation(), .2, .01, 4)
        a = f.step(Observation(), .5, .01, 5)
        self.assertEqual(a.state,"STOPPED")

    def test_search_timeout_latches_even_when_line_returns(self):
        f = Follower(config())
        acquire(f, .4)
        f.step(Observation(), .2, .01, 4)
        a = f.step(Observation(), 4.3, .01, 5)
        self.assertEqual(a.state,"STOPPED")
        for i in range(6,12):
            a = f.step(obs(), 4.3+i/30, .01, i)
        self.assertEqual((a.left,a.right),(0,0))

    def test_reacquire_debounced(self):
        f = Follower(config())
        acquire(f, .4)
        f.step(Observation(), .2, .01, 4)
        for i in range(5,8):
            a = f.step(obs(.2), .3+(i-5)/30, .01, i)
            self.assertEqual((a.left,a.right),(0,0))
        a = f.step(obs(.2), .4, .01, 8)
        self.assertEqual(a.state, "TRACK")
        self.assertGreater(a.left+a.right, 0)

    def test_reacquisition_respects_minimum_stop_time(self):
        f = Follower(config())
        acquire(f, .4)
        f.step(Observation(), .2, .01, 4)
        for i in range(5,9):
            a = f.step(obs(.2), .2+(i-4)*.03, .01, i)
            self.assertEqual((a.left,a.right),(0,0))
        a = f.step(obs(.2), .45, .01, 9)
        self.assertEqual(a.state,"TRACK")

    def test_camera_timeout_latches(self):
        f = Follower(config())
        acquire(f,.4)
        a = f.step(obs(.4), .2, .4, 4)
        self.assertEqual(a.state,"FAULT")
        b = f.step(obs(), .3, .01, 5)
        self.assertEqual((b.left,b.right),(0,0))

    def test_ambiguous_junction_stops(self):
        f = Follower(config())
        acquire(f)
        a = f.step(Observation(ambiguous=True), .2, .01, 4)
        self.assertEqual(a.state,"STOPPED")

    def test_persistent_large_error_stops(self):
        f = Follower(config())
        acquire(f, .95)
        for i in range(4, 80):
            a = f.step(obs(.95), i/30, .01, i)
        self.assertEqual(a.state,"STOPPED")
        self.assertEqual(a.reason,"persistent_tracking_error")

    def test_repeated_losses_limited(self):
        f = Follower(config())
        acquire(f,.4)
        seq, now = 4, .2
        for attempt in range(4):
            a = f.step(Observation(), now, .01, seq)
            seq += 1
            if attempt < 3:
                for _ in range(4):
                    now += .06
                    f.step(obs(.4), now, .01, seq)
                    seq += 1
            now += .1
        self.assertEqual(a.state,"STOPPED")
        self.assertEqual(a.reason,"too_many_line_losses")

    def test_wheel_saturation_preserves_ratio(self):
        a,b = wheel_speeds(.3,.8,.16,.18)
        self.assertLessEqual(max(abs(a),abs(b)),.18)
        self.assertAlmostEqual(a/b,(.3+.8*.08)/(.3-.8*.08))

    def test_radius_limit_does_not_reverse_tracking_wheels(self):
        f = Follower(config())
        acquire(f,1)
        for i in range(4,30):
            a = f.step(obs(1),i/30,.01,i)
            self.assertGreaterEqual(min(a.left,a.right),-1e-10)
            self.assertLessEqual(max(a.left,a.right),.18)


class MotorAndConfigTests(unittest.TestCase):
    def test_drive_requires_measured_geometry_and_confirmation(self):
        cfg = config()
        with self.assertRaises(ValueError):
            validate_config(cfg,True)
        cfg["hardware"]["confirmed"] = True
        with self.assertRaises(ValueError):
            validate_config(cfg,True)
        cfg["geometry"]["wheel_track_m"] = .165
        validate_config(cfg,True)

    def test_duplicate_pin_rejected(self):
        cfg = config()
        cfg["hardware"]["right"]["pwm"] = cfg["hardware"]["left"]["pwm"]
        with self.assertRaises(ValueError):
            validate_config(cfg)

    def test_invalid_calibration_rejected(self):
        cfg = config()
        cfg["hardware"]["left_speed_duty"][1][1] = .9
        with self.assertRaises(ValueError):
            validate_config(cfg)

    def test_pwm_zero_and_limit(self):
        hw=config()["hardware"]
        self.assertEqual(speed_to_duty(0,hw["left_speed_duty"],.65),0)
        self.assertGreater(speed_to_duty(.1,hw["left_speed_duty"],.65),0)
        self.assertLess(speed_to_duty(-.1,hw["left_speed_duty"],.65),0)
        self.assertLessEqual(speed_to_duty(99,hw["left_speed_duty"],.65),.65)

    def test_software_watchdog_latches(self):
        cfg=config()
        cfg["control"]["command_watchdog_s"] = .06
        m=MotorOutput(cfg)
        try:
            m.command(.1,.1)
            deadline=time.monotonic()+.5
            while not m.fault and time.monotonic()<deadline:
                time.sleep(.01)
            self.assertIsNotNone(m.fault)
            self.assertEqual(m.duties,(0,0))
            with self.assertRaises(RuntimeError):
                m.command(.1,.1)
        finally:
            m.close()

    def test_mock_gpio_directions_standby_and_cleanup(self):
        from gpiozero import Device
        from gpiozero.pins.mock import MockFactory, MockPWMPin
        old=Device.pin_factory
        Device.pin_factory=MockFactory(pin_class=MockPWMPin)
        cfg=config()
        cfg["hardware"]["driver"]="TB6612FNG"
        cfg["hardware"]["standby_pin"]=16
        m=MotorOutput(cfg,drive=True)
        try:
            m.command(.1,-.1)
            self.assertTrue(m.standby.value)
            self.assertTrue(m.channels[0].a.value)
            self.assertFalse(m.channels[0].b.value)
            self.assertFalse(m.channels[1].a.value)
            self.assertTrue(m.channels[1].b.value)
            m.stop()
            self.assertFalse(m.standby.value)
            self.assertEqual(m.channels[0].pwm.value,0)
            self.assertEqual(m.channels[1].pwm.value,0)
        finally:
            m.close()
            Device.pin_factory=old


if __name__ == "__main__":
    unittest.main(verbosity=2)
