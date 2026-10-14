#!/usr/bin/env python3
"""
Raspberry Pi OpenCV Line Follower
---------------------------------
Hardware: Raspberry Pi (3B+/4/5), L298N-style motor driver, 2 DC motors,
          USB webcam, 20 x 25 cm differential-drive chassis.

Run:
    python3 line_follower.py            # normal run (headless, fastest)
    python3 line_follower.py --view     # show debug window (slower, for tuning)
    python3 line_follower.py --dry      # vision only, motors disabled
    python3 line_follower.py --speed 0.5
    python3 line_follower.py --stream   # browser monitor at http://<PI_IP>:5000
    python3 line_follower.py --test-motors

Design:
  * Camera thread always keeps ONLY the newest frame (no buffer lag).
  * Image split into horizontal bands -> line position at several distances.
  * Positions converted to cm -> pure pursuit curvature -> wheel speeds using
    the real track width, so turning matches the chassis geometry.
  * State machine: FOLLOW -> COAST -> SEARCH (last side) -> SEARCH (other side) -> STOP
"""

import argparse
import math
import signal
import threading
import time

import logging

import cv2
import numpy as np

try:
    from flask import Flask, Response
    HAVE_FLASK = True
except ImportError:
    HAVE_FLASK = False

try:
    import RPi.GPIO as GPIO
    HAVE_GPIO = True
except (ImportError, RuntimeError):
    HAVE_GPIO = False


# ============================================================
# 1. MOTOR / GPIO CONFIG (BCM) - same pins as your diagnostic
# ============================================================
IN1, IN2, EN1 = 4, 17, 23      # left motor
IN3, IN4, EN2 = 27, 22, 24     # right motor

PWM_FREQ = 1000          # Hz. 1 kHz is smoother than 100 Hz for small DC motors
MIN_DUTY = 30            # duty % where the wheel JUST starts turning (measure it!)
MAX_DUTY = 100
LEFT_TRIM = 1.00         # reduce the faster motor (e.g. 0.95) if robot drifts on straights
RIGHT_TRIM = 1.00
LEFT_REVERSED = False    # flip if a wheel spins backwards on "forward"
RIGHT_REVERSED = False


# ============================================================
# 2. CHASSIS GEOMETRY  (20 x 25 cm chassis)
# ============================================================
TRACK_WIDTH_CM = 17.0    # distance between LEFT and RIGHT wheel centres (measure)
R_MIN_CM = 6.0           # tightest radius allowed. < TRACK/2 means inner wheel reverses
CAMERA_LATERAL_OFFSET_CM = 0.0   # + if camera is mounted left of centre


# ============================================================
# 3. CAMERA CONFIG
# ============================================================
CAM_INDEX = 0
FRAME_W, FRAME_H = 320, 240      # small = fast. Do not go above 320x240 on a Pi
CAM_FPS = 30
USE_MJPG = True                  # most USB webcams only reach 30 fps in MJPG
MANUAL_EXPOSURE = False          # True = no brightness pumping + less motion blur
EXPOSURE_VALUE = 150             # camera dependent; lower = darker, less blur
FRAME_STALE_S = 0.30             # stop motors if newest frame is older than this


# ============================================================
# 4. VISION CONFIG
# ============================================================
LINE_IS_DARK = True              # black tape on light floor
ROI_TOP_FRAC = 0.35              # ignore the top 35% of the image (too far / horizon)
N_BANDS = 4                      # bands from bottom (near) to top (far)
BLUR_K = 5
USE_OTSU = True                  # automatic threshold, adapts to lighting
OTSU_LIMIT = 110                 # dark line: threshold never above this (stops floor
                                 # noise being detected as line when no line visible)
FIXED_THRESH = 80                # used when USE_OTSU = False
MIN_FILL = 0.35                  # a column counts as line if >=35% of band rows are line
MIN_SEG_PX = 4                   # ignore segments narrower than this (noise)
MAX_SEG_FRAC = 0.55              # wider than 55% of frame = crossing/junction -> skip band
MAX_JUMP_PX = 70                 # max x jump between adjacent bands (rejects other lines)

# Camera-to-floor calibration, one entry per band, BOTTOM band first.
#   BAND_DIST_CM : forward distance from the WHEEL AXLE to that band on the floor
#   CM_PER_PX    : how many cm one pixel spans sideways at that band
# Measure with a ruler on the floor (see instructions in the chat). Defaults are
# typical for a camera ~15 cm high, tilted ~40 deg down.
BAND_DIST_CM = [20.0, 26.0, 33.0, 42.0]
CM_PER_PX = [0.070, 0.085, 0.100, 0.120]
BAND_WEIGHTS = [0.15, 0.30, 0.35, 0.20]   # which distances steer the robot most


# ============================================================
# 5. CONTROL CONFIG
# ============================================================
BASE_SPEED = 0.70          # normalised 0..1 on straights
MIN_SPEED_FRAC = 0.35      # never slower than 35% of base while following
CURVE_SLOWDOWN = 18.0      # bigger = slows more in curves (kappa is 1/cm)
K_PP = 1.0                 # steering aggressiveness multiplier (0.7 .. 1.5)
KAPPA_SMOOTH = 0.6         # EMA weight of new curvature (1 = no smoothing)
EDGE_MARGIN_PX = 30        # line this close to image edge -> strong side memory

# Line-lost behaviour
LOST_COAST_S = 0.25        # keep last steering this long (gaps, glitches)
COAST_SPEED_FRAC = 0.6
SEARCH_SPEED = 0.45        # pivot speed (normalised)
SEARCH_FIRST_S = 1.2       # pivot towards last seen side
SEARCH_SECOND_S = 2.4      # then sweep the other way (passes origin + extra)
FOUND_CONFIRM = 2          # frames line must be seen before resuming after search

# Browser monitor (--stream). Encoding runs in its own thread, never in the
# control loop, so streaming cannot delay steering decisions.
STREAM_PORT = 5000
STREAM_FPS = 8
STREAM_JPEG_QUALITY = 60


# ============================================================
# MOTORS
# ============================================================
class Motors:
    def __init__(self, dry=False):
        self.dry = dry or not HAVE_GPIO
        if self.dry:
            print("[motors] DRY mode - no GPIO output")
            return
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for p in (IN1, IN2, IN3, IN4, EN1, EN2):
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
        self.lp = GPIO.PWM(EN1, PWM_FREQ)
        self.rp = GPIO.PWM(EN2, PWM_FREQ)
        self.lp.start(0)
        self.rp.start(0)

    @staticmethod
    def _to_duty(u):
        """Normalised -1..1 -> duty %, with deadband compensation."""
        a = abs(u)
        if a < 0.03:
            return 0.0
        return MIN_DUTY + (MAX_DUTY - MIN_DUTY) * min(a, 1.0)

    def _one(self, u, pa, pb, pwm, reversed_):
        if reversed_:
            u = -u
        if u > 0:
            GPIO.output(pa, GPIO.HIGH); GPIO.output(pb, GPIO.LOW)
        elif u < 0:
            GPIO.output(pa, GPIO.LOW); GPIO.output(pb, GPIO.HIGH)
        else:
            GPIO.output(pa, GPIO.LOW); GPIO.output(pb, GPIO.LOW)
        pwm.ChangeDutyCycle(self._to_duty(u))

    def drive(self, left, right):
        """left/right normalised -1..1. Scales BOTH down if one exceeds 1
        so the left/right ratio (= turn radius) is preserved."""
        left *= LEFT_TRIM
        right *= RIGHT_TRIM
        m = max(abs(left), abs(right), 1.0)
        left, right = left / m, right / m
        if self.dry:
            return
        self._one(left, IN1, IN2, self.lp, LEFT_REVERSED)
        self._one(right, IN3, IN4, self.rp, RIGHT_REVERSED)

    def stop(self):
        if self.dry:
            return
        self.lp.ChangeDutyCycle(0)
        self.rp.ChangeDutyCycle(0)
        for p in (IN1, IN2, IN3, IN4):
            GPIO.output(p, GPIO.LOW)

    def close(self):
        if self.dry:
            return
        self.stop()
        self.lp.stop()
        self.rp.stop()
        GPIO.cleanup()


# ============================================================
# LOW-LATENCY CAMERA (always newest frame)
# ============================================================
class Camera:
    def __init__(self):
        self.cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("Camera not found")
        if USE_MJPG:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        self.cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if MANUAL_EXPOSURE:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # V4L2: 1 = manual
            self.cap.set(cv2.CAP_PROP_EXPOSURE, EXPOSURE_VALUE)

        self.cond = threading.Condition()
        self.frame, self.ts, self.fid = None, 0.0, 0
        self.running = True
        self.t = threading.Thread(target=self._loop, daemon=True)
        self.t.start()

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self.cond:
                self.frame, self.ts = f, time.monotonic()
                self.fid += 1
                self.cond.notify_all()

    def wait_new(self, last_fid, timeout=0.2):
        """Block until a frame newer than last_fid exists."""
        with self.cond:
            self.cond.wait_for(lambda: self.fid != last_fid, timeout=timeout)
            return self.frame, self.ts, self.fid

    def close(self):
        self.running = False
        self.t.join(timeout=1)
        self.cap.release()


# ============================================================
# VISION
# ============================================================
KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))


def segments(col_counts, min_count):
    on = (col_counts >= min_count).astype(np.int8)
    d = np.diff(np.concatenate(([0], on, [0])))
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0]))


def find_line(frame, prev_bottom_x):
    """Returns list of (band_idx, cx_px, cy_px) from bottom band upward, and mask."""
    h, w = frame.shape[:2]
    y0 = int(h * ROI_TOP_FRAC)
    gray = cv2.cvtColor(frame[y0:], cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (BLUR_K, BLUR_K), 0)

    flag = cv2.THRESH_BINARY_INV if LINE_IS_DARK else cv2.THRESH_BINARY
    if USE_OTSU:
        t, _ = cv2.threshold(gray, 0, 255, flag | cv2.THRESH_OTSU)
        t = min(t, OTSU_LIMIT) if LINE_IS_DARK else max(t, 255 - OTSU_LIMIT)
    else:
        t = FIXED_THRESH
    _, mask = cv2.threshold(gray, t, 255, flag)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)

    rh = mask.shape[0]
    bh = rh // N_BANDS
    min_count = max(1, int(bh * MIN_FILL))
    pts = []
    predict_x = prev_bottom_x if prev_bottom_x is not None else w / 2

    for i in range(N_BANDS):                     # 0 = bottom
        y_hi = rh - i * bh
        y_lo = y_hi - bh
        band = mask[y_lo:y_hi]
        counts = np.count_nonzero(band, axis=0)
        best, best_d = None, 1e9
        for s, e in segments(counts, min_count):
            width = e - s
            if width < MIN_SEG_PX or width > MAX_SEG_FRAC * w:
                continue
            idx = np.arange(s, e)
            cx = float(np.dot(idx, counts[s:e]) / counts[s:e].sum())
            dist = abs(cx - predict_x)
            if dist < best_d:
                best, best_d = cx, dist
        if best is None:
            continue                              # crossing or gap in this band
        if pts and best_d > MAX_JUMP_PX:
            break                                 # jumped to another line: stop
        pts.append((i, best, y0 + (y_lo + y_hi) / 2))
        predict_x = best
    return pts, mask, y0


def curvature_from_points(pts, w):
    """Pure pursuit: weighted target point (cm) -> path curvature (1/cm, + = left)."""
    sw = sx = sy = 0.0
    for i, cx, _ in pts:
        lat = (w / 2 - cx) * CM_PER_PX[i] + CAMERA_LATERAL_OFFSET_CM
        fwd = BAND_DIST_CM[i]
        wt = BAND_WEIGHTS[i]
        sx += wt * lat; sy += wt * fwd; sw += wt
    lat, fwd = sx / sw, sy / sw
    kappa = K_PP * 2.0 * lat / (lat * lat + fwd * fwd)
    kmax = 1.0 / R_MIN_CM
    return max(-kmax, min(kmax, kappa)), lat


def wheels_for(v, kappa):
    """Differential drive kinematics with real track width."""
    half = TRACK_WIDTH_CM / 2.0
    return v * (1.0 - kappa * half), v * (1.0 + kappa * half)


# ============================================================
# MAIN LOOP
# ============================================================
def draw_debug(frame, mask, pts, state, kappa, y0):
    """Camera view with overlays + full-size mask, side by side."""
    h, w = frame.shape[:2]
    view = frame.copy()
    cv2.line(view, (0, y0), (w, y0), (255, 0, 0), 1)
    rh = h - y0
    bh = rh // N_BANDS
    for i in range(1, N_BANDS):
        y = h - i * bh
        cv2.line(view, (0, y), (w, y), (80, 80, 80), 1)
    cv2.line(view, (w // 2, h), (w // 2, y0), (0, 0, 255), 1)
    for _, cx, cy in pts:
        cv2.circle(view, (int(cx), int(cy)), 5, (0, 255, 0), -1)
    cv2.putText(view, f"{state} k={kappa:+.3f}", (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    full = np.zeros((h, w), np.uint8)
    full[y0:y0 + mask.shape[0]] = mask
    return np.hstack((view, cv2.cvtColor(full, cv2.COLOR_GRAY2BGR)))


class Streamer:
    """MJPEG browser monitor. publish() only stores a reference (microseconds);
    drawing + JPEG encoding happen in a separate thread at STREAM_FPS."""

    PAGE = """<!DOCTYPE html><html><head><title>Line Follower</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{margin:0;padding:16px;background:#151517;color:#eee;
font-family:Arial,sans-serif;text-align:center}
img{width:100%;max-width:960px;border:1px solid #444}</style></head>
<body><h2>Line Follower Monitor</h2><img src="/video_feed">
<p>Left: camera (green = detected line per band) | Right: mask</p></body></html>"""

    def __init__(self, port=STREAM_PORT, fps=STREAM_FPS):
        self.lock = threading.Lock()
        self.src = None
        self.jpeg, self.jid = None, 0
        self.period = 1.0 / fps
        threading.Thread(target=self._encode_loop, daemon=True).start()

        logging.getLogger("werkzeug").setLevel(logging.ERROR)
        app = Flask(__name__)
        app.add_url_rule("/", "index", lambda: self.PAGE)
        app.add_url_rule("/video_feed", "feed", self._feed)
        threading.Thread(
            target=lambda: app.run(host="0.0.0.0", port=port, debug=False,
                                   use_reloader=False, threaded=True),
            daemon=True).start()
        print(f"[stream] http://<PI_IP>:{port}")

    def publish(self, frame, mask, pts, state, kappa, y0):
        with self.lock:
            self.src = (frame, mask, list(pts), state, kappa, y0)

    def _encode_loop(self):
        while True:
            time.sleep(self.period)
            with self.lock:
                src, self.src = self.src, None
            if src is None:
                continue
            img = draw_debug(*src)
            ok, j = cv2.imencode(".jpg", img,
                                 [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_JPEG_QUALITY])
            if ok:
                with self.lock:
                    self.jpeg, self.jid = j.tobytes(), self.jid + 1

    def _feed(self):
        def gen():
            last = -1
            while True:
                with self.lock:
                    data, jid = self.jpeg, self.jid
                if data is None or jid == last:
                    time.sleep(0.03)
                    continue
                last = jid
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n")
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame",
                        headers={"Cache-Control": "no-cache, no-store"})


def run_motor_test(motors, u=0.4, secs=1.5):
    steps = [("LEFT forward", (u, 0)), ("LEFT reverse", (-u, 0)),
             ("RIGHT forward", (0, u)), ("RIGHT reverse", (0, -u)),
             ("BOTH forward", (u, u)),
             ("PIVOT left", (-u, u)), ("PIVOT right", (u, -u))]
    print("MOTOR TEST - lift the wheels off the floor.")
    for name, (l, r) in steps:
        input(f"ENTER to run {name} ...")
        motors.drive(l, r)
        time.sleep(secs)
        motors.stop()
        time.sleep(0.4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="show debug window")
    ap.add_argument("--dry", action="store_true", help="disable motors")
    ap.add_argument("--speed", type=float, default=BASE_SPEED)
    ap.add_argument("--stream", action="store_true", help="browser monitor")
    ap.add_argument("--test-motors", action="store_true")
    args = ap.parse_args()

    cv2.setUseOptimized(True)
    motors = Motors(dry=args.dry)

    if args.test_motors:
        try:
            run_motor_test(motors)
        except KeyboardInterrupt:
            pass
        finally:
            motors.close()
        return

    cam = Camera()
    streamer = None
    if args.stream:
        if HAVE_FLASK:
            streamer = Streamer()
        else:
            print("[stream] Flask not installed: pip install flask")

    stop_flag = {"stop": False}
    signal.signal(signal.SIGTERM, lambda *_: stop_flag.update(stop=True))

    state = "FOLLOW"
    kappa_f = 0.0
    last_side = 0            # +1 left, -1 right, 0 unknown
    last_seen = time.monotonic()
    prev_bottom_x = None
    found_count = 0
    last_fid = 0
    n_frames, t_stat = 0, time.monotonic()

    print("Waiting for camera...")
    while cam.fid == 0:
        time.sleep(0.05)
    print("Running. Ctrl+C to stop.")

    try:
        while not stop_flag["stop"]:
            frame, ts, fid = cam.wait_new(last_fid)
            now = time.monotonic()
            if fid == last_fid or frame is None or now - ts > FRAME_STALE_S:
                motors.stop()                         # camera stalled: safety stop
                continue
            last_fid = fid
            h, w = frame.shape[:2]

            pts, mask, y0 = find_line(frame, prev_bottom_x)

            if pts:
                kappa, lat = curvature_from_points(pts, w)
                bottom_x = pts[0][1] if pts[0][0] == 0 else None

                # ---- remember which side the line is on ----
                if bottom_x is not None and bottom_x < EDGE_MARGIN_PX:
                    last_side = +1
                elif bottom_x is not None and bottom_x > w - EDGE_MARGIN_PX:
                    last_side = -1
                elif abs(lat) > 1.0:
                    last_side = 1 if lat > 0 else -1

                if state in ("SEARCH1", "SEARCH2", "STOP"):
                    found_count += 1
                    motors.stop()                     # stop dead while confirming
                    if found_count >= FOUND_CONFIRM:
                        state, kappa_f, found_count = "FOLLOW", kappa, 0
                else:
                    state = "FOLLOW"
                    kappa_f = KAPPA_SMOOTH * kappa + (1 - KAPPA_SMOOTH) * kappa_f
                    # slow down for curves and when only part of the line is visible
                    v = args.speed / (1.0 + CURVE_SLOWDOWN * abs(kappa_f))
                    v *= 0.6 + 0.4 * (len(pts) / N_BANDS)
                    v = max(v, args.speed * MIN_SPEED_FRAC)
                    motors.drive(*wheels_for(v, kappa_f))

                last_seen = now
                prev_bottom_x = bottom_x if bottom_x is not None else pts[0][1]

            else:
                found_count = 0
                lost = now - last_seen
                side = last_side if last_side != 0 else 1
                if lost < LOST_COAST_S:
                    state = "COAST"
                    motors.drive(*wheels_for(args.speed * COAST_SPEED_FRAC, kappa_f))
                elif lost < LOST_COAST_S + SEARCH_FIRST_S:
                    state = "SEARCH1"
                    motors.drive(-side * SEARCH_SPEED, side * SEARCH_SPEED)
                elif lost < LOST_COAST_S + SEARCH_FIRST_S + SEARCH_SECOND_S:
                    state = "SEARCH2"
                    motors.drive(side * SEARCH_SPEED, -side * SEARCH_SPEED)
                else:
                    if state != "STOP":
                        print("Line lost - stopped. Place robot on line to resume.")
                    state = "STOP"
                    motors.stop()
                prev_bottom_x = None

            # ---- stats ----
            n_frames += 1
            if now - t_stat >= 1.0:
                print(f"{n_frames:3d} fps | {state:7s} | kappa {kappa_f:+.3f} "
                      f"| bands {len(pts)} | lag {(now - ts) * 1000:4.0f} ms")
                n_frames, t_stat = 0, now

            # ---- optional debug view ----
            if streamer is not None:
                streamer.publish(frame, mask, pts, state, kappa_f, y0)
            if args.view:
                cv2.imshow("view", draw_debug(frame, mask, pts, state, kappa_f, y0))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        motors.close()
        cam.close()
        if args.view:
            cv2.destroyAllWindows()
        print("Stopped, GPIO cleaned up.")


if __name__ == "__main__":
    main()
