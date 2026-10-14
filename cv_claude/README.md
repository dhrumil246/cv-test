# Raspberry Pi OpenCV Line Follower

A camera-based line-following robot for a Raspberry Pi with a USB webcam, an L298N-style motor driver and a 20 × 25 cm two-wheel differential-drive chassis.

The robot looks at the line at four distances ahead. It converts what it sees into a steering curve that matches the chassis's real wheel spacing, and it slows down in curves. If it loses the line, it recovers on its own: it coasts briefly, searches toward the side where the line was last seen, sweeps back the other way, and finally stops.

---

## Quick start

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-rpi.gpio python3-flask

python3 line_follower.py --test-motors      # 1. check wiring (wheels lifted)
python3 line_follower.py --dry --stream     # 2. check vision, motors off
python3 line_follower.py --stream --speed 0.4   # 3. first run, slow
```

Open `http://<PI_IP>:5000` in a browser on the same network to watch the live camera and mask. To find the Pi's IP address, run `hostname -I`.

---

## 1. Hardware

| Part | Notes |
|---|---|
| Raspberry Pi 3B+ / 4 / 5 | A Pi 4 or 5 is recommended for a steady 25–30 fps |
| USB webcam | Must support MJPG at 320×240 (almost all do) |
| L298N (or similar dual H-bridge) | |
| 2 × DC gear motors + wheels, 1 caster | |
| Motor battery | Separate from the Pi's supply (e.g. 2S Li-ion, 7.4 V) |
| Pi power | 5 V, 3 A power bank or buck converter |
| Track | Black tape (about 18–25 mm wide) on a light, matte floor |

## 2. Wiring

All pin numbers in the code are **BCM** numbers.

| Driver pin | Raspberry Pi GPIO (BCM) | Physical pin | Purpose |
|---|---|---|---|
| IN1 | GPIO 4 | 7 | Left motor direction A |
| IN2 | GPIO 17 | 11 | Left motor direction B |
| ENA | GPIO 23 | 16 | Left motor speed (PWM) |
| IN3 | GPIO 27 | 13 | Right motor direction A |
| IN4 | GPIO 22 | 15 | Right motor direction B |
| ENB | GPIO 24 | 18 | Right motor speed (PWM) |
| GND | GND | 6 | **Common ground. This is required.** |

Important:

- Remove the ENA/ENB jumpers on the L298N. Otherwise the motors always run at full speed and ignore the PWM signal.
- Connect the driver's GND to the Pi's GND. Without a shared ground, the motors behave erratically.
- Never power the motors from the Pi's 5 V pin.
- The L298N drops about 2 V. Make sure the motor battery voltage, minus 2 V, still comes close to the motors' rated voltage.

## 3. Camera mounting

This matters as much as the code.

- **Height:** 12–18 cm above the floor, at the front of the chassis.
- **Tilt:** 35–45° downward.
- **Field of view:** the bottom of the image should land about 5 cm ahead of the front bumper. The top of the area the code uses (the blue line in the debug view) should reach 35–45 cm ahead.
- **Alignment:** mount the camera centred and pointing straight forward. If it can't be centred, set `CAMERA_LATERAL_OFFSET_CM`.
- **Rigidity:** the mount must not wobble. Any vibration shows up directly as steering noise.

A camera pointing straight down sees only a few centimetres of line. On a curve the line leaves the frame within a frame or two, and the robot loses it.

## 4. Software install

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-rpi.gpio python3-flask
```

**Raspberry Pi 5:** RPi.GPIO does not support the Pi 5. Install the drop-in replacement instead:

```bash
sudo apt remove -y python3-rpi.gpio
sudo apt install -y python3-rpi-lgpio
```

To check that the camera is detected, run:

```bash
ls /dev/video*
v4l2-ctl --list-formats-ext     # optional: sudo apt install v4l-utils
```

If the webcam isn't `/dev/video0`, change `CAM_INDEX` in the config.

## 5. Command-line options

| Command | What it does |
|---|---|
| `python3 line_follower.py` | Normal run. No display, so it runs fastest. |
| `--stream` | Browser monitor at `http://<PI_IP>:5000` (runs in its own thread) |
| `--view` | Local OpenCV window. This is slow; use it only with a monitor attached. |
| `--dry` | Vision only, with the motors disabled |
| `--speed 0.5` | Base speed from 0 to 1 (default 0.70) |
| `--test-motors` | Step-by-step wheel direction test |

Stop the robot with **Ctrl+C**. The motors stop and the GPIO pins are cleaned up automatically.

---

## 6. Calibration (do this once, in order)

### 6.1 Motor direction

```bash
python3 line_follower.py --test-motors
```

Lift the wheels off the floor before starting. Each step should move the wheels as its name says. If a wheel turns the wrong way on a "forward" step, set `LEFT_REVERSED = True` or `RIGHT_REVERSED = True`. You don't need to swap any wires.

### 6.2 Minimum duty (`MIN_DUTY`)

Put the robot on the floor. Raise `TEST_SPEED` in your old diagnostic script until both wheels just start moving under load, and set `MIN_DUTY` to that value. It's typically 22–35. If it's too low, the inner wheel stalls on curves. If it's too high, the robot jerks.

### 6.3 Track width (`TRACK_WIDTH_CM`)

Measure the distance between the centre of the left wheel and the centre of the right wheel. The default of 17.0 cm suits a 20 cm wide chassis.

### 6.4 Camera-to-floor calibration (`BAND_DIST_CM`, `CM_PER_PX`)

This step converts pixels into centimetres, so the steering matches your real turning geometry.

1. Run `python3 line_follower.py --dry --stream` and open the browser monitor.
2. The grey lines in the camera view divide the image into 4 bands. Band 0 is at the bottom, nearest the robot.
3. For each band, from bottom to top:
   - Put a ruler on the floor at that band's centre. Measure the forward distance from the **wheel axle** to it. That value goes in `BAND_DIST_CM`.
   - Lay a ruler across the image at that band. Count how many pixels 10 cm covers, then set `CM_PER_PX = 10 / pixels`. You can do this from a screenshot of the stream; the camera half of the stream is 320 pixels wide.
4. Enter the four values for each list in order, bottom band first:

```python
BAND_DIST_CM = [20.0, 26.0, 33.0, 42.0]
CM_PER_PX    = [0.070, 0.085, 0.100, 0.120]
```

Each value should increase from the bottom band to the top band. If one doesn't, recheck that measurement.

### 6.5 Straight-line trim

Drive along a straight line. If the robot steadily drifts to one side, lower the trim value of the faster wheel slightly (for example `LEFT_TRIM = 0.95`).

### 6.6 Vision check

In the stream, check the following:

- The mask (right side) shows the tape as a clean white stripe, with no noise from the floor.
- A green dot appears on the line in every band.
- When you lift the robot off the line, all dots disappear and the terminal shows the robot going into `COAST` and then `SEARCH`.

If the floor shows up as noise, lower `OTSU_LIMIT` (for example to 90). If the tape breaks up, raise it. If the line breaks up when the robot moves, set `MANUAL_EXPOSURE = True` and lower `EXPOSURE_VALUE` to reduce motion blur.

---

## 7. Tuning

Start with `--speed 0.4`. Change one setting at a time.

| Symptom | Change |
|---|---|
| Wobbles or zig-zags on straights | Lower `K_PP` (to about 0.8) or lower `KAPPA_SMOOTH` (to about 0.4) |
| Runs wide on curves | Raise `K_PP` (to about 1.2) or raise `CURVE_SLOWDOWN` |
| Too slow overall in curves | Lower `CURVE_SLOWDOWN` or raise `MIN_SPEED_FRAC` |
| Loses the line on sharp corners | Lower the speed, raise `CURVE_SLOWDOWN`, tilt the camera up a little |
| Cuts corners on hairpins | Lower `R_MIN_CM` so the robot is allowed to turn tighter |
| Gets distracted by other lines nearby | Lower `MAX_JUMP_PX` |
| Stops at small gaps in the tape | Raise `LOST_COAST_S` (to about 0.4) |
| Overshoots the line while searching | Lower `SEARCH_SPEED` |

When the robot handles curves reliably, increase `--speed` in steps of 0.1.

### Health check

The terminal prints one status line per second, for example:

```
 28 fps | FOLLOW  | kappa +0.021 | bands 4 | lag   38 ms
```

Aim for **at least 20 fps** and a **lag under 60 ms**. If you see less, run without `--view`, check that the camera is delivering MJPG, and confirm that nothing else heavy is running on the Pi.

---

## 8. How it behaves

| State | When | What the robot does |
|---|---|---|
| `FOLLOW` | Line visible | Steers along the line and slows in curves |
| `COAST` | Line lost for less than 0.25 s | Keeps the last steering at reduced speed. This covers gaps and single bad frames. |
| `SEARCH1` | Line lost for 0.25 to 1.45 s | Pivots toward the side where the line was last seen |
| `SEARCH2` | Line lost for 1.45 to 3.85 s | Sweeps the other way, back past the starting heading |
| `STOP` | Line lost after that | Stops and waits. Put the robot back on the line and it resumes. |

When the line reappears during a search, the robot stops and waits until it has seen the line in 2 consecutive frames before following again. This prevents overshoot.

A camera watchdog stops the motors if no new frame arrives within 0.3 s.

---

## 9. Run at boot (optional)

Create `/etc/systemd/system/linefollower.service`. Change the paths and username to match your setup.

```ini
[Unit]
Description=Line follower
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/line_follower.py --stream
WorkingDirectory=/home/pi
User=pi
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Then enable it:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now linefollower
journalctl -u linefollower -f     # view the log
```

Running at boot means the robot starts driving as soon as it powers up. Only enable this once tuning is finished.

---

## 10. Troubleshooting

| Problem | Likely cause and fix |
|---|---|
| `Camera not found` | Wrong `CAM_INDEX`, or the camera isn't connected. Check `ls /dev/video*`. |
| Motors don't move at all | The ENA/ENB jumpers are still fitted, there's no common ground, the motor battery is flat, or `MIN_DUTY` is too low |
| One wheel always runs at full speed | That driver's ENA/ENB jumper is still fitted |
| The Pi reboots when the motors start | The Pi and the motors share a power supply. Use separate supplies with a common ground. |
| Very low fps or high lag | `--view` is on, the camera isn't delivering MJPG, or you're on an old Pi. Use `--stream` instead of `--view`. |
| The line flickers in the mask | Uneven lighting or glare. Use a matte floor, add diffuse lighting, or lock the exposure. |
| `RuntimeError` mentioning GPIO on a Pi 5 | Install `python3-rpi-lgpio` (see section 4) |
| Browser page doesn't load | You're on a different network, or port 5000 is blocked. Check the address with `hostname -I`. |
