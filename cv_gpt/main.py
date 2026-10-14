#!/usr/bin/env python3
"""USB-camera line follower for a differential-drive Raspberry Pi robot.

Dry-run is the default. Read README.md before using --drive.
Positive image error/curvature/yaw means RIGHT throughout this program.
Wheel speeds are estimates from calibrated PWM tables, not encoder feedback.
"""
import argparse
from collections import deque
import csv
from dataclasses import dataclass, field
import json
import logging
import math
from pathlib import Path
import signal
import threading
import time

import cv2
import numpy as np

LOG = logging.getLogger("line_follower")


def clip(value, low, high):
    return max(low, min(high, value))


def load_config(path, drive=False):
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    validate_config(cfg, drive)
    return cfg


def validate_config(cfg, drive=False):
    def finite_tree(obj):
        if isinstance(obj, dict):
            for v in obj.values():
                finite_tree(v)
        elif isinstance(obj, list):
            for v in obj:
                finite_tree(v)
        elif isinstance(obj, (int, float)) and not math.isfinite(obj):
            raise ValueError("Configuration contains a non-finite number")

    finite_tree(cfg)
    hw, geo, cam, vis, ctl, rec = [cfg[k] for k in
        ("hardware", "geometry", "camera", "vision", "control", "recovery")]
    if hw["driver"] not in ("L298N", "TB6612FNG"):
        raise ValueError("Supported driver interfaces: L298N or TB6612FNG")
    pins = [hw[side][key] for side in ("left", "right") for key in ("in1", "in2", "pwm")]
    if hw["standby_pin"] is not None:
        pins.append(hw["standby_pin"])
    if any(type(p) is not int or not 0 <= p <= 27 for p in pins) or len(set(pins)) != len(pins):
        raise ValueError("GPIO pins must be distinct BCM numbers from 0 to 27")
    if hw["driver"] == "TB6612FNG" and hw["standby_pin"] is None:
        raise ValueError("Set standby_pin for TB6612FNG")
    if not 0 < hw["max_duty"] <= 1 or not 10 <= hw["pwm_hz"] <= 20000:
        raise ValueError("Invalid PWM limit/frequency")
    for side in ("left", "right"):
        table = hw[side + "_speed_duty"]
        if len(table) < 3 or table[0] != [0.0, 0.0]:
            raise ValueError("Each speed/duty table must start with [0, 0] and have >=3 rows")
        for prev, curr in zip(table, table[1:]):
            if len(curr) != 2 or not (curr[0] > prev[0] and prev[1] < curr[1] <= hw["max_duty"]):
                raise ValueError("Speed and duty tables must increase; duty must not exceed max_duty")
        if table[-1][0] < ctl["max_wheel_speed_mps"]:
            raise ValueError("Speed/duty table must cover max_wheel_speed_mps")
    track = geo["wheel_track_m"]
    if track is not None and not 0.05 <= track <= 0.50:
        raise ValueError("Measure wheel_track_m in metres; expected 0.05 to 0.50")
    if not 0.05 <= geo["preview_wheel_track_m"] <= 0.50:
        raise ValueError("Invalid preview_wheel_track_m")
    if drive and (hw["confirmed"] is not True or track is None):
        raise ValueError("Before --drive: verify wiring, set hardware.confirmed=true, and measure geometry.wheel_track_m")
    if not (160 <= cam["width"] <= 1920 and 120 <= cam["height"] <= 1080 and 1 <= cam["fps"] <= 120):
        raise ValueError("Invalid camera size/FPS")
    if cam["fourcc"] and len(cam["fourcc"]) != 4:
        raise ValueError("fourcc must be empty or four characters")
    if not 0.05 <= cam["max_frame_age_s"] <= 1 or cam["startup_timeout_s"] <= 0:
        raise ValueError("Invalid camera timeout")
    bands = vis["bands_y"]
    if len(bands) < 3 or bands != sorted(set(bands), reverse=True):
        raise ValueError("bands_y needs >=3 distinct values, ordered near to far (descending)")
    if not all(vis["roi_top"] < y < 0.99 for y in bands) or not 0 <= vis["roi_top"] < 0.8:
        raise ValueError("Bands must lie inside the ground ROI")
    if vis["line_color"] not in ("black", "white") or vis["threshold_mode"] not in ("otsu", "fixed", "adaptive"):
        raise ValueError("Invalid line color or threshold mode")
    if vis["adaptive_block_size"] < 3 or vis["adaptive_block_size"] % 2 != 1:
        raise ValueError("adaptive_block_size must be odd and >=3")
    if not 3 <= vis["min_valid_bands"] <= len(bands) or not 0 < vis["min_confidence"] <= 1:
        raise ValueError("Invalid detection confidence/band requirement")
    if not 0 < vis["max_line_width_fraction"] < 0.8 or vis["min_line_width_px"] < 1:
        raise ValueError("Invalid line widths")
    if not 0 < vis["max_band_step_fraction"] <= 1 or vis["band_half_height_px"] < 1:
        raise ValueError("Invalid band geometry")
    if not 0 <= vis["near_weight"] <= 1 or not 0 <= vis["center_reference_fraction"] <= 1:
        raise ValueError("Invalid vision weights/reference")
    if not 0 < ctl["min_speed_mps"] <= ctl["base_speed_mps"] <= ctl["max_wheel_speed_mps"] <= 0.5:
        raise ValueError("Require 0 < min speed <= base speed <= max wheel speed <= 0.5 m/s")
    positive = ("max_curvature_per_m", "max_yaw_rate_radps", "wheel_accel_mps2",
                "command_watchdog_s", "persistent_error_timeout_s")
    if any(ctl[k] <= 0 for k in positive) or ctl["acquire_frames"] < 2:
        raise ValueError("Invalid control limits")
    if any(ctl[k] < 0 for k in ("kp_curvature", "kd_curvature", "derivative_filter_tau_s", "curve_slowdown")):
        raise ValueError("Controller gains/time constants must be nonnegative")
    if rec["brake_s"] < 0 or rec["search_leg_s"] <= 0 or rec["timeout_s"] <= rec["brake_s"]:
        raise ValueError("Invalid recovery timing")
    if rec["search_yaw_rate_radps"] <= 0 or rec["max_attempts"] < 1 or rec["attempt_window_s"] <= 0:
        raise ValueError("Invalid recovery limits")


@dataclass
class Observation:
    valid: bool = False
    error: float = 0.0
    bend: float = 0.0
    confidence: float = 0.0
    ambiguous: bool = False
    reason: str = "line_not_found"
    points: list = field(default_factory=list)


class LineDetector:
    """Trace contiguous line candidates through narrow bands, bottom to top."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.last_near = None

    def detect(self, frame):
        v = self.cfg
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        top = int(v["roi_top"] * h)
        ground = cv2.GaussianBlur(gray[top:], (3, 3), 0)
        # Max-min of the smoothed ROI also works when thin tape occupies <1%.
        if int(ground.max()) - int(ground.min()) < v["min_contrast"]:
            return Observation(reason="low_contrast"), np.zeros_like(gray)
        polarity = cv2.THRESH_BINARY_INV if v["line_color"] == "black" else cv2.THRESH_BINARY
        mode = v["threshold_mode"]
        if mode == "adaptive":
            # Invert white tape so the same local-dark-feature rule applies.
            local = ground if v["line_color"] == "black" else 255 - ground
            mask_roi = cv2.adaptiveThreshold(local, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV, v["adaptive_block_size"], v["adaptive_c"])
        else:
            flag = polarity | (cv2.THRESH_OTSU if mode == "otsu" else 0)
            _, mask_roi = cv2.threshold(ground, v["fixed_threshold"], 255, flag)
        mask = np.zeros_like(gray)
        mask[top:] = mask_roi
        points, multi_bands, wide_bands = [], 0, 0
        anchor = self.last_near if self.last_near is not None else w * v["center_reference_fraction"]
        for yf in v["bands_y"]:
            y = int(yf * h)
            half = v["band_half_height_px"]
            band = mask[max(top, y-half):min(h, y+half+1)]
            occupied = np.mean(band > 0, axis=0) >= 0.45
            edges = np.diff(np.pad(occupied.astype(np.int8), (1, 1)))
            starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
            candidates = []
            for a, b in zip(starts, ends):
                width = b-a
                if width > w * v["max_line_width_fraction"]:
                    wide_bands += 1
                elif width >= v["min_line_width_px"]:
                    candidates.append((a+b-1)/2)
            if len(candidates) > 1:
                multi_bands += 1
            if not candidates:
                continue
            x = min(candidates, key=lambda candidate: abs(candidate-anchor))
            if points:
                # Allow a larger horizontal displacement when a band was missed.
                gap = (points[-1][1]/h - yf) / (v["bands_y"][0]-v["bands_y"][1])
                if abs(x-anchor) > w * v["max_band_step_fraction"] * max(1.0, gap):
                    continue
            points.append((float(x), float(y)))
            anchor = x
        ambiguous = wide_bands > 0 or multi_bands >= 2
        if ambiguous:
            return Observation(ambiguous=True, reason="junction_or_wide_marking", points=points), mask
        if len(points) < v["min_valid_bands"] or points[0][1]/h < v["min_nearest_y"]:
            self.last_near = None
            return Observation(reason="insufficient_line", points=points), mask
        xs = np.array([p[0]/w for p in points])
        ys = np.array([p[1]/h for p in points])
        curve = np.polyfit(ys, xs, 2)
        residual = float(np.sqrt(np.mean((np.polyval(curve, ys)-xs)**2)))
        confidence = len(points)/len(v["bands_y"]) * max(0.0, 1.0-residual/0.08)
        if confidence < v["min_confidence"]:
            return Observation(confidence=confidence, reason="inconsistent_line", points=points), mask
        # Evaluate only INSIDE observed bands, never extrapolate a missing far path.
        ref = v["center_reference_fraction"]
        near = (float(np.polyval(curve, clip(0.85, ys.min(), ys.max())))-ref)*2
        mid = (float(np.polyval(curve, clip(0.65, ys.min(), ys.max())))-ref)*2
        far = (float(np.polyval(curve, clip(0.45, ys.min(), ys.max())))-ref)*2
        heading = far-near
        error = clip(v["near_weight"]*near + (1-v["near_weight"])*mid + v["heading_weight"]*heading, -1, 1)
        bend = max(abs(heading), abs(near-2*mid+far))
        self.last_near = points[0][0]
        return Observation(True, error, bend, confidence, False, "line", points), mask


def wheel_speeds(v, right_yaw, track, maximum):
    """v in m/s, clockwise yaw in rad/s; preserve wheel ratio on saturation."""
    left, right = v + right_yaw*track/2, v - right_yaw*track/2
    scale = max(1.0, abs(left)/maximum, abs(right)/maximum)
    return left/scale, right/scale


@dataclass
class Command:
    state: str
    left: float = 0.0
    right: float = 0.0
    reason: str = ""
    curvature: float = 0.0


class Follower:
    """Pure state machine; one step per distinct frame. Stops are immediate."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.c, self.r = cfg["control"], cfg["recovery"]
        self.track = cfg["geometry"]["wheel_track_m"] or cfg["geometry"]["preview_wheel_track_m"]
        self.state = "WAIT_LINE"
        self.reason = "waiting_for_stable_line"
        self.acquired = 0
        self.last_seq = -1
        self.previous_time = None
        self.previous_error = None
        self.derivative = 0.0
        self.last_command = (0.0, 0.0)
        self.last_direction = 0
        self.lost_since = None
        self.attempts = deque()
        self.bad_error_since = None

    def halt(self, reason, fault=False):
        self.state = "FAULT" if fault else "STOPPED"
        self.reason = reason
        return self.zero()

    def zero(self):
        self.last_command = (0.0, 0.0)
        self.previous_error = None
        self.derivative = 0.0
        return Command(self.state, reason=self.reason)

    def step(self, obs, now, frame_age, sequence):
        if self.state in ("STOPPED", "FAULT"):
            return self.zero()
        if frame_age > self.cfg["camera"]["max_frame_age_s"] or frame_age < 0:
            return self.halt("camera_frame_timeout", fault=True)
        if sequence == self.last_seq:
            return Command(self.state, *self.last_command, reason="waiting_for_new_frame")
        self.last_seq = sequence
        dt = 1/30 if self.previous_time is None else clip(now-self.previous_time, 0.001, 0.25)
        self.previous_time = now
        if obs.ambiguous:
            return self.halt("junction_or_wide_marking")
        good = obs.valid and obs.confidence >= self.cfg["vision"]["min_confidence"]
        if self.state == "WAIT_LINE":
            self.acquired = self.acquired+1 if good else 0
            if self.acquired < self.c["acquire_frames"]:
                return self.zero()
            self.state, self.reason = "TRACK", "line_acquired"
        if self.state == "TRACK" and not good:
            self.state, self.reason = "BRAKE", obs.reason
            self.lost_since = now
            self.acquired = 0
            self.bad_error_since = None
            while self.attempts and now-self.attempts[0] > self.r["attempt_window_s"]:
                self.attempts.popleft()
            self.attempts.append(now)
            if len(self.attempts) > self.r["max_attempts"]:
                return self.halt("too_many_line_losses")
            return self.zero()
        if self.state in ("BRAKE", "SEARCH"):
            elapsed = now-self.lost_since
            # An absolute timeout still applies if glimpses repeatedly appear.
            if elapsed >= self.r["timeout_s"]:
                return self.halt("line_recovery_timeout")
            self.acquired = self.acquired+1 if good else 0
            if self.acquired >= self.c["acquire_frames"] and elapsed >= self.r["brake_s"]:
                self.state, self.reason = "TRACK", "line_reacquired"
                self.previous_error = None
            elif good:
                self.reason = "verifying_reacquisition"
                return self.zero()
            elif elapsed < self.r["brake_s"]:
                return self.zero()
            elif not self.r["enabled"] or self.last_direction == 0:
                return self.halt("line_lost_no_search_direction")
            else:
                self.state, self.reason = "SEARCH", "bounded_pivot_search"
                # +direction for one leg, -direction for two, +direction for one.
                # This searches both sides of the loss pose. No blind forward/backward drive.
                t, leg = elapsed-self.r["brake_s"], self.r["search_leg_s"]
                direction = self.last_direction * (1 if t < leg or t >= 3*leg else -1)
                if abs(t-leg) < 0.10 or abs(t-3*leg) < 0.10:
                    return self.zero()  # brief zero command at reversal
                omega = direction * min(self.r["search_yaw_rate_radps"], self.c["max_yaw_rate_radps"])
                wheels = wheel_speeds(0, omega, self.track, self.c["max_wheel_speed_mps"])
                return self.ramp(wheels, dt)
        if abs(obs.error) > self.c["persistent_error_threshold"]:
            if self.bad_error_since is None:
                self.bad_error_since = now
            elif now-self.bad_error_since >= self.c["persistent_error_timeout_s"]:
                return self.halt("persistent_tracking_error")
        else:
            self.bad_error_since = None
        raw_d = 0.0 if self.previous_error is None else (obs.error-self.previous_error)/dt
        alpha = dt/(self.c["derivative_filter_tau_s"]+dt)
        self.derivative += alpha*(raw_d-self.derivative)
        self.previous_error = obs.error
        # kappa is an IMAGE-ERROR CONTROL COMMAND, not measured track curvature.
        limit = min(self.c["max_curvature_per_m"], 2/self.track)
        kappa = clip(self.c["kp_curvature"]*obs.error + self.c["kd_curvature"]*self.derivative, -limit, limit)
        if abs(obs.error) > 0.06:
            self.last_direction = 1 if obs.error > 0 else -1
        elif abs(kappa) < 0.5:
            self.last_direction = 0
        speed = self.c["base_speed_mps"] / (1+self.c["curve_slowdown"]*abs(kappa)+1.5*obs.bend)
        speed *= clip(obs.confidence, 0.5, 1)
        speed = max(self.c["min_speed_mps"], speed)
        if abs(kappa) > 1e-6:
            speed = min(speed, self.c["max_yaw_rate_radps"]/abs(kappa))
        wheels = wheel_speeds(speed, speed*kappa, self.track, self.c["max_wheel_speed_mps"])
        return self.ramp(wheels, dt, kappa)

    def ramp(self, wheels, dt, kappa=0.0):
        # Scale the change vector together; avoid independently clipping each wheel.
        delta = [target-old for target, old in zip(wheels, self.last_command)]
        largest = max(abs(x) for x in delta)
        scale = min(1.0, self.c["wheel_accel_mps2"]*dt/largest) if largest else 1.0
        self.last_command = tuple(old+scale*d for old, d in zip(self.last_command, delta))
        return Command(self.state, *self.last_command, self.reason, kappa)


def speed_to_duty(speed, table, maximum):
    """No arbitrary minimum-duty boost: tiny commands must not become big turns."""
    value = float(np.interp(abs(speed), [p[0] for p in table], [p[1] for p in table]))
    return math.copysign(min(maximum, value), speed) if abs(speed) > 1e-6 else 0.0


class HBridgeChannel:
    def __init__(self, cfg, frequency):
        from gpiozero import DigitalOutputDevice, PWMOutputDevice
        self.devices = []
        self.sign = 0
        self.invert = cfg["invert"]
        try:
            self.pwm = PWMOutputDevice(cfg["pwm"], frequency=frequency, initial_value=0)
            self.devices.append(self.pwm)
            self.a = DigitalOutputDevice(cfg["in1"], initial_value=False)
            self.devices.append(self.a)
            self.b = DigitalOutputDevice(cfg["in2"], initial_value=False)
            self.devices.append(self.b)
        except Exception:
            self.close()
            raise

    def write(self, duty):
        duty *= -1 if self.invert else 1
        sign = 1 if duty > 0 else -1 if duty < 0 else 0
        if sign != self.sign:
            self.pwm.value = 0
            self.a.off()
            self.b.off()
            if sign and self.sign:
                time.sleep(0.005)
            self.a.value = sign > 0
            self.b.value = sign < 0
            self.sign = sign
        self.pwm.value = abs(duty)

    def close(self):
        for device in self.devices:
            try:
                device.off()
            except Exception:
                pass
        for device in reversed(self.devices):
            device.close()


class MotorOutput:
    """Software watchdog latches OFF after missed commands; not a hardware E-stop."""
    def __init__(self, cfg, drive=False):
        self.hw, self.limit = cfg["hardware"], cfg["control"]["command_watchdog_s"]
        self.drive = drive
        self.channels = []
        self.standby = None
        self.lock = threading.Lock()
        self.finished = threading.Event()
        self.fault = None
        self.last_update = time.monotonic()
        self.duties = (0.0, 0.0)
        self.thread = None
        try:
            if drive:
                if self.hw["standby_pin"] is not None:
                    from gpiozero import DigitalOutputDevice
                    self.standby = DigitalOutputDevice(self.hw["standby_pin"], initial_value=False)
                for side in ("left", "right"):
                    self.channels.append(HBridgeChannel(self.hw[side], self.hw["pwm_hz"]))
            self.thread = threading.Thread(target=self._watch, daemon=True)
            self.thread.start()
        except Exception:
            self.close()
            raise

    def _write(self, duties):
        if self.standby is not None and any(duties):
            self.standby.on()
        for channel, duty in zip(self.channels, duties):
            channel.write(duty)
        if self.standby is not None and not any(duties):
            self.standby.off()
        self.duties = duties

    def command(self, left, right):
        if not math.isfinite(left) or not math.isfinite(right):
            raise ValueError("Non-finite motor command")
        duties = tuple(speed_to_duty(speed, self.hw[side+"_speed_duty"], self.hw["max_duty"])
                       for side, speed in zip(("left", "right"), (left, right)))
        return self.command_duty(*duties)

    def command_duty(self, left, right):
        duties = (left, right)
        if any(not math.isfinite(x) or abs(x) > self.hw["max_duty"] for x in duties):
            raise ValueError("Duty exceeds configured limit or is not finite")
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            self._write(duties)
            self.last_update = time.monotonic()
            return duties

    def _watch(self):
        while not self.finished.wait(0.025):
            with self.lock:
                if any(self.duties) and time.monotonic()-self.last_update > self.limit:
                    self.fault = "motor_command_watchdog_timeout"
                    self._write((0.0, 0.0))
                    LOG.error(self.fault)

    def stop(self):
        with self.lock:
            # Attempt both channels even if one GPIO operation raises.
            for channel in self.channels:
                try:
                    channel.write(0.0)
                except Exception:
                    LOG.exception("Could not stop a motor output")
            if self.standby is not None:
                self.standby.off()
            self.duties = (0.0, 0.0)

    def close(self):
        self.finished.set()
        if self.thread:
            self.thread.join(timeout=0.5)
        try:
            self.stop()
        finally:
            if self.standby is not None:
                self.standby.off()
            for channel in self.channels:
                channel.close()
            if self.standby is not None:
                self.standby.close()


class LatestCamera:
    """Single overwrite slot; timestamps are host receipt times, NOT sensor times."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.finished = threading.Event()
        self.latest = None
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        cap = None
        try:
            cap = cv2.VideoCapture(self.cfg["device"], cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError("Cannot open USB camera with V4L2")
            if self.cfg["fourcc"]:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.cfg["fourcc"]))
            for prop, value in ((cv2.CAP_PROP_FRAME_WIDTH, self.cfg["width"]),
                                (cv2.CAP_PROP_FRAME_HEIGHT, self.cfg["height"]),
                                (cv2.CAP_PROP_FPS, self.cfg["fps"]),
                                (cv2.CAP_PROP_BUFFERSIZE, 1)):
                if not cap.set(prop, value):
                    LOG.warning("Camera did not accept property %s=%s", prop, value)
            LOG.info("Camera reported %.0fx%.0f @ %.1f FPS; measure actual latency separately",
                     cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT), cap.get(cv2.CAP_PROP_FPS))
            seq = 0
            while not self.finished.is_set():
                ok, frame = cap.read()
                stamp = time.monotonic()
                if not ok or frame is None:
                    raise RuntimeError("Camera read failed/disconnected")
                if self.cfg["flip_horizontal"]:
                    frame = cv2.flip(frame, 1)
                if self.cfg["flip_vertical"]:
                    frame = cv2.flip(frame, 0)
                seq += 1
                with self.lock:
                    self.latest = (seq, stamp, frame)
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
        finally:
            if cap is not None:
                cap.release()

    def get(self):
        with self.lock:
            return self.latest, self.error

    def close(self):
        self.finished.set()
        # Do not release a capture object concurrently with its blocking read().
        self.thread.join(timeout=1.0)


def annotate(frame, mask, obs, cmd, cfg, age):
    view = frame.copy()
    h, w = view.shape[:2]
    for yf in cfg["vision"]["bands_y"]:
        cv2.line(view, (0, int(yf*h)), (w-1, int(yf*h)), (120, 100, 0), 1)
    cx = int(w*cfg["vision"]["center_reference_fraction"])
    cv2.line(view, (cx, 0), (cx, h-1), (255, 100, 0), 1)
    for x, y in obs.points:
        cv2.circle(view, (int(x), int(y)), 5, (0, 220, 0), -1)
    lines = [f"{cmd.state}: {cmd.reason}",
             f"error={obs.error:+.2f} conf={obs.confidence:.2f} age={age*1000:.0f}ms",
             f"L={cmd.left:+.3f} R={cmd.right:+.3f} m/s (estimated)"]
    for i, text in enumerate(lines):
        cv2.putText(view, text, (8, 22+24*i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    return np.hstack((view, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default=str(Path(__file__).with_name("config.json")))
    p.add_argument("--drive", action="store_true", help="Enable GPIO after configuring hardware and track width")
    p.add_argument("--show", action="store_true", help="Open live image/mask window (q to quit)")
    p.add_argument("--log", default="run.csv", help="CSV telemetry path")
    p.add_argument("--video", help="Replay a local recording at its FPS; drive is forbidden")
    p.add_argument("--max-seconds", type=float, default=0, help="Optional maximum runtime; 0 = unlimited")
    p.add_argument("--motor-test", choices=("left", "right", "forward", "reverse", "turn-right"))
    p.add_argument("--test-seconds", type=float, default=1.0, help="Motor test duration, capped at 2 seconds")
    p.add_argument("--test-duty", type=float, help="Motor test only: use raw PWM magnitude for calibration")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    if args.video and args.drive:
        p.error("--video cannot be combined with --drive")
    if args.motor_test and args.video:
        p.error("--motor-test cannot be combined with --video")
    if args.test_duty is not None and (not args.motor_test or not math.isfinite(args.test_duty) or args.test_duty <= 0):
        p.error("--test-duty needs --motor-test and a positive finite value")
    cfg = load_config(args.config, args.drive)
    if not args.drive:
        LOG.info("DRY RUN: GPIO is not accessed; wheel commands are displayed/logged only")
    if cfg["geometry"]["wheel_track_m"] is None:
        LOG.warning("Preview uses an UNMEASURED %.3f m track; set the actual wheel centre spacing before driving",
                    cfg["geometry"]["preview_wheel_track_m"])
    quit_event = threading.Event()
    def request_stop(_signum, _frame):
        quit_event.set()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    motors = camera = replay = logfile = None
    exit_code = 0
    try:
        motors = MotorOutput(cfg, args.drive)
        if args.motor_test:
            speed = min(0.06, cfg["control"]["base_speed_mps"])
            pairs = {"left": (speed, 0), "right": (0, speed), "forward": (speed, speed),
                     "reverse": (-speed, -speed), "turn-right": (speed, -speed)}
            deadline = time.monotonic()+clip(args.test_seconds, 0.1, 2.0)
            LOG.info("Motor test %s: %s m/s (estimated)", args.motor_test, pairs[args.motor_test])
            if args.test_duty is not None:
                LOG.info("Overriding estimated speed with raw duty %.3f", args.test_duty)
            while not quit_event.is_set() and time.monotonic() < deadline:
                if args.test_duty is None:
                    motors.command(*pairs[args.motor_test])
                else:
                    motors.command_duty(*(math.copysign(args.test_duty, x) if x else 0.0
                                          for x in pairs[args.motor_test]))
                quit_event.wait(0.02)
            return 0
        detector, follower = LineDetector(cfg["vision"]), Follower(cfg)
        if args.video:
            replay = cv2.VideoCapture(args.video)
            if not replay.isOpened():
                raise RuntimeError("Cannot open replay file")
            replay_fps = replay.get(cv2.CAP_PROP_FPS)
            if not math.isfinite(replay_fps) or replay_fps <= 0:
                replay_fps = cfg["camera"]["fps"]
        else:
            camera = LatestCamera(cfg["camera"])
        logfile = open(args.log, "w", newline="", encoding="utf-8")
        writer = csv.writer(logfile)
        writer.writerow(["elapsed_s", "frame", "state", "reason", "error", "bend", "confidence",
                         "frame_age_ms", "processing_ms", "left_mps_est", "right_mps_est", "left_duty", "right_duty", "curvature_command"])
        started = time.monotonic()
        last_seq, next_replay, report_at, old_state = -1, started, started, None
        while not quit_event.is_set():
            now = time.monotonic()
            if args.max_seconds > 0 and now-started >= args.max_seconds:
                break
            if motors.fault:
                raise RuntimeError(motors.fault)
            if replay:
                if now < next_replay:
                    quit_event.wait(min(0.01, next_replay-now))
                    continue
                ok, frame = replay.read()
                if not ok:
                    LOG.info("Replay ended")
                    break
                seq, stamp = last_seq+1, time.monotonic()
                next_replay = stamp+1/replay_fps
                if cfg["camera"]["flip_horizontal"]:
                    frame = cv2.flip(frame, 1)
                if cfg["camera"]["flip_vertical"]:
                    frame = cv2.flip(frame, 0)
            else:
                item, error = camera.get()
                if error:
                    raise RuntimeError(error)
                if item is None:
                    if now-started > cfg["camera"]["startup_timeout_s"]:
                        raise RuntimeError("Camera startup timeout")
                    quit_event.wait(0.005)
                    continue
                seq, stamp, frame = item
                if now-stamp > cfg["camera"]["max_frame_age_s"]:
                    raise RuntimeError("Camera stopped delivering fresh frames")
                if seq == last_seq:
                    quit_event.wait(0.002)
                    continue
            last_seq = seq
            process_start = time.monotonic()
            obs, mask = detector.detect(frame)
            now = time.monotonic()
            age = now-stamp  # checked after processing, before any motor command
            cmd = follower.step(obs, now, age, seq)
            duties = motors.command(cmd.left, cmd.right)
            writer.writerow([round(now-started, 4), seq, cmd.state, cmd.reason, round(obs.error, 4),
                round(obs.bend, 4), round(obs.confidence, 3), round(age*1000, 2),
                round((now-process_start)*1000, 2), round(cmd.left, 4), round(cmd.right, 4),
                round(duties[0], 3), round(duties[1], 3), round(cmd.curvature, 3)])
            if now >= report_at or cmd.state != old_state:
                LOG.info("%s | %s | e=%+.2f confidence=%.2f L=%+.3f R=%+.3f", cmd.state, cmd.reason,
                         obs.error, obs.confidence, cmd.left, cmd.right)
                logfile.flush()
                report_at, old_state = now+1, cmd.state
            if args.show:
                cv2.imshow("Line follower: camera | mask", annotate(frame, mask, obs, cmd, cfg, age))
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            if cmd.state in ("STOPPED", "FAULT"):
                LOG.warning("Stopped: %s. Fix/reposition and restart to re-arm.", cmd.reason)
                exit_code = 2
                break
    except Exception:
        LOG.exception("Stopped after an error")
        exit_code = 1
    finally:
        # Remove drive BEFORE potentially blocking camera/file cleanup.
        if motors:
            motors.close()
        if camera:
            camera.close()
        if replay:
            replay.release()
        if logfile:
            logfile.close()
        if args.show:
            cv2.destroyAllWindows()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
