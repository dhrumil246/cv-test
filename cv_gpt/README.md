# Raspberry Pi camera line follower

A configurable Python/OpenCV controller for a small robot with independently driven left and right wheels and a USB webcam. Assumed chassis orientation: **20 cm across, 25 cm front to back**. The program defaults to **dry-run: no GPIO access**.

Updated to your supplied scripts: left side BCM **4/17/23**, right side BCM **27/22/24**, PWM **100 Hz**. For your four-motor chassis, one command controls the front and rear motors on each side, assuming they are wired to the same driver channel. This is two motor channels, not four independent motor outputs. The model of the driver and loaded motor response cannot be identified from pin assignments alone.

This is a tested software starting point, not a claim of seamless operation on untested hardware. Wiring, camera view, wheel spacing and motor response must be matched to your robot. A steering-servo car requires a different motor/steering adapter and different kinematics.

## What is included

- `line_follower.py`: camera capture, detection, steering, motor output, recovery and telemetry.
- `config.json`: all pin assignments, geometry, vision settings and tuning values.
- `test_line_follower.py`: synthetic-image, controller and mocked-GPIO tests.
- `generate_demo.py`: makes a synthetic recording for a dry-run demonstration.
- `VALIDATION.md`: test results and what has not been tested.
- `CODE_REVIEW.md`: findings from your uploaded line follower and pasted motor diagnostic, with reproduced numerical examples.

## 1. Install on Raspberry Pi OS

Use a current Raspberry Pi OS installation and its system Python. The USB webcam is read through Linux V4L2; a ribbon CSI camera is not implemented by this capture adapter.

```bash
sudo apt update
sudo apt install python3-opencv python3-numpy python3-gpiozero python3-lgpio v4l-utils
cd raspberry_pi_line_follower
python3 line_follower.py --show
```

On an SSH terminal without a desktop, omit `--show`. Watch the terminal and `run.csv`. A camera read failure prints an error and exits. Press Ctrl+C, or `q` in the display window, to stop.

GPIO Zero's modern default backend supports lgpio, including Pi 5. If backend selection fails, check the installed packages, your membership of the `gpio` group, and try `GPIOZERO_PIN_FACTORY=lgpio python3 line_follower.py ...`. Do not switch blindly to RPi.GPIO on Pi 5.

## 2. Match the motor driver and wiring

This implementation uses **two direction pins plus one PWM/enable pin per motor channel**. It supports the common L298N and TB6612FNG interfaces, not arbitrary motor HATs, I2C controllers, DRV8833 wiring, servos or ESCs.

All values in `config.json` are **BCM GPIO numbers**, not physical header numbers.

| Function | BCM GPIO | Physical header pin | L298N terminal | TB6612FNG terminal |
|---|---:|---:|---|---|
| Left direction 1 | 4 | 7 | IN1 | AIN1 |
| Left direction 2 | 17 | 11 | IN2 | AIN2 |
| Left PWM | 23 | 16 | ENA | PWMA |
| Right direction 1 | 27 | 13 | IN3 | BIN1 |
| Right direction 2 | 22 | 15 | IN4 | BIN2 |
| Right PWM | 24 | 18 | ENB | PWMB |
| Optional standby | 16 | 36 | Not used | STBY |
| Common ground | GND | e.g. 6 | GND | GND |

For L298N, the supplied config has `driver: "L298N"` and `standby_pin: null`. Remove ENA/ENB enable jumpers if your module uses them, so GPIO controls those inputs. The module's separate logic-power/regulator jumper is a different setting: follow that module's documentation.

L298N remains an interface assumption, not a confirmed identification from your scripts. GPIO Zero is still used for output; importing RPi.GPIO is not necessary to preserve your BCM pins or 100 Hz PWM. Other peripherals must not claim the same pins (for example, an enabled one-wire bus can use GPIO4).

For TB6612FNG, change `driver` to `"TB6612FNG"` and `standby_pin` to `16`. Use the appropriate logic supply and motor supply for your board. Check board current limits against the motors, particularly for a four-motor chassis; this program cannot detect overcurrent.

Power motors through the driver from an appropriate motor supply. Use a properly regulated Pi supply, share signal ground, and do not connect motors directly to GPIO. GPIO is 3.3 V logic. The exact power wiring depends on the driver module, so the table above covers control signals only.

After verifying your actual wiring, set `hardware.confirmed` to `true`. The program refuses `--drive` until this is set and wheel spacing is measured. This is an explicit operating-mode guard in the supplied program, not a request for remote approval.

## 3. Set geometry correctly

Measure **centre-to-centre distance between the left and right driven wheel contact tracks**. Put that distance in metres in `geometry.wheel_track_m`; for example, 16.5 cm becomes `0.165`. Do not enter the full chassis width unless it actually equals the wheel spacing.

The supplied `null` deliberately means unknown. `preview_wheel_track_m: 0.16` is used only for dry-run calculations and is not an inferred measurement.

For ideal differential drive:

```text
left wheel speed  = forward speed + clockwise yaw rate × wheel spacing / 2
right wheel speed = forward speed - clockwise yaw rate × wheel spacing / 2
centre-path radius magnitude = |forward speed / yaw rate|
```

During normal tracking both wheels remain forward or stopped. The curvature command is limited to `min(max_curvature_per_m, 2 / wheel_track_m)`. This means the ideal minimum centre-path radius is at least `wheel_track_m / 2`, potentially larger if the configured curvature limit is lower. With an EXAMPLE measured 0.16 m spacing, the kinematic floor would be 0.08 m. This is not the clearance needed by the whole chassis and not a guarantee of achieved radius.

Recovery can pivot by reversing one wheel, giving an ideal zero centre-path radius. The body still sweeps space. Even a 20 × 25 cm rectangle rotating about its centre reaches about 16 cm from that centre at its corners; an offset axle, wheels, webcam and brackets increase or change the swept area. Chassis length and width are recorded for reference, not used to invent steering geometry.

Four-wheel skid-steer robots experience tyre scrub, so actual turning differs from the ideal equations. This controller has no encoders: requested wheel speeds and turning radius are estimates from the PWM calibration. For accurate motion across changing loads/battery levels, add wheel encoders and closed-loop wheel-speed controllers.

## 4. Check motor direction and calibrate response

Lift the wheels for the initial direction tests:

```bash
python3 line_follower.py --drive --motor-test left --test-seconds 1
python3 line_follower.py --drive --motor-test right --test-seconds 1
python3 line_follower.py --drive --motor-test forward --test-seconds 1
python3 line_follower.py --drive --motor-test turn-right --test-seconds 1
```

If one side goes backwards, change only that side's `invert` setting. For `turn-right`, the left side should go forward and the right side backward. Check that any motors connected to the same side agree physically.

The supplied `left_speed_duty` and `right_speed_duty` tables are **illustrative estimates**, not specifications of your motors. Each row is `[wheel_ground_speed_mps, PWM_duty_from_0_to_1]`. Measure your motors at several duty values under representative load and replace the tables separately for the two sides. Keep `[0,0]` first and both columns increasing. The final speed must cover `max_wheel_speed_mps`; duty must stay within `max_duty`.

You can command a known raw PWM duty for a short test:

```bash
python3 line_follower.py --drive --motor-test forward --test-duty 0.35 --test-seconds 2
```

Use measured wheel RPM and wheel circumference, or controlled distance/time measurements, to estimate speed. Startup acceleration biases a short distance/time measurement; measure the steady portion if possible. A lifted-wheel test establishes direction, not loaded speed calibration. The test duration is capped at two seconds. Increase duty gradually if the wheels do not start, keeping within your hardware's limits.

At speeds below the first calibrated moving point, PWM is interpolated towards zero. This avoids silently forcing an inside wheel to a large minimum speed, but it may stall. If low-speed commands do not move reliably, adjust calibration, speed settings or gearing. Reverse recovery uses the same magnitude calibration as forward; verify reverse behaviour separately with `--motor-test reverse`.

## 5. Mount and configure the camera

- Mount rigidly near the chassis centreline, angled down and forward. All five sample bands should show ground.
- Keep both nearby tape and the approaching curve visible. Looking only directly beneath the bumper causes late steering.
- Start at 640 × 480, 30 FPS, MJPG. Check `v4l2-ctl --list-devices` and `v4l2-ctl -d /dev/video0 --list-formats-ext` for supported modes. Match `camera.device` to the video device, e.g. `0` or `"/dev/video2"`.
- If MJPG is unsupported, set `fourcc` to `"YUYV"` if supported, or `""` to let the camera choose. Inspect reported size/FPS and measure actual latency; successful property writes do not prove a latency target.
- For focus/exposure controls, use `v4l2-ctl -d /dev/video0 --list-ctrls-menus`. Names, ranges and manual-mode values vary. Under stable light, lock focus at ground distance and try short exposure with adequate light. No camera-specific control values are blindly written by this program.
- Adjust `flip_horizontal`/`flip_vertical` only to make the camera view match the robot's world orientation. Physically moving tape right must produce a positive error and a right turn command.

The capture thread continuously replaces one frame slot so the processing loop does not build an application queue. It requests a one-frame backend buffer, but the backend may ignore this. Frame age in the CSV is measured **since host receipt**, not sensor exposure. Upstream USB/driver latency and cameras repeating old images with new reads are not detected by this timestamp. Compare a visible moving object/LED with the preview to check end-to-end latency.

## 6. Verify detection before enabling motion

```bash
python3 line_follower.py --show --log dry_run.csv
```

Move the robot by hand through the track, including its tightest bends. On the left is the camera view with sample bands and selected points. On the right is the binary mask, where the tape should be white. Wheel commands are simulated.

Defaults assume **black tape on a light floor**. For the opposite, set `line_color: "white"`. The five bands at 90%, 78%, 66%, 54%, and 42% image height are traced from near to far. A quadratic fit estimates a smooth image-space path; nearby offset and farther direction create the steering error. This is an image-based controller, not metric reconstruction of the floor.

| Setting | When to change it |
|---|---|
| `threshold_mode: "otsu"` | Good first choice for a distinct dark/light scene |
| `threshold_mode: "adaptive"` | Try for gradients or uneven lighting |
| `adaptive_block_size: 61` | Increase if adaptive thresholding splits broad tape into two edges; should exceed tape width substantially |
| `threshold_mode: "fixed"`, `fixed_threshold` | Use for controlled lighting after inspecting the mask |
| `min_line_width_px` | Reduce only if real distant tape is too thin to detect |
| `max_line_width_fraction` | Increase if legitimate tape or a sharp diagonal fills too much of a band; overly large values accept unwanted blobs |
| `bands_y`, `roi_top` | Keep sample bands on ground and within useful near/far coverage |
| `center_reference_fraction` | Adjust if the optical centreline is not the desired robot path centre |
| `min_valid_bands`, `min_confidence` | Keep conservative until true line detections are stable |

Multiple branches in two or more bands, or a very broad marking, trigger a stop. This deliberately treats forks/crossings/finish bars as ambiguous, because no route-selection rule has been supplied. Detection is heuristic: it cannot reliably distinguish every shadow, nearby parallel line, sharp corner or junction. A false positive can still steer the robot incorrectly. Check the actual track.

## 7. Run at low speed, then tune

After the dry-run, wiring and geometry checks:

```bash
python3 line_follower.py --drive --show --log first_drive.csv --max-seconds 20
```

The starting requested forward speed is 0.10 m/s, reducing toward 0.04 m/s on bends. These are estimates until motor calibration is done. The maximum requested wheel speed is 0.18 m/s, and duty is capped at 0.65. These caps do not establish real physical speed without calibration.

Your old script's `BASE_SPEED=26` means 26% PWM, not 0.26 m/s. The new illustrative table requests roughly 42% duty at 0.10 m/s, so matching pins and frequency does NOT match the old physical speed. Calibrate first; do not assume the replacement is slower. Your old 40% direction check can be reproduced with `--motor-test forward --test-duty 0.40 --test-seconds 1.5` (with `--drive` and the configuration checks complete).

1. Tune broad curves first. If steering begins too late, fix the camera view and farther bands before raising gains.
2. If it consistently turns too widely while detection is right, lower base speed, inspect inside-wheel response, and gradually increase `kp_curvature` (initially 12). Check whether the curvature command already hits its limit.
3. If it oscillates, lower proportional gain or gradually adjust `kd_curvature` (initially 0.35). Derivative filtering starts at 0.10 s. No integral term is used, so there is no integral windup.
4. `curve_slowdown` controls slowing with steering demand; the image bend estimate also slows the robot. Raise slowdown if entry speed is excessive.
5. `wheel_accel_mps2` limits command changes. Too small a value delays steering; too large can cause jerks. Stops bypass the ramp.
6. Change one setting at a time, compare `run.csv`, and increase speed only after left and right curves are reliable.

Do not expect a moving car to follow an arbitrarily sharp 90-degree corner at normal speed. The image must retain enough path and the chassis must be able to turn. This implementation slows, may lose the line, then attempts its bounded recovery; it does not include a dedicated intersection/90-degree route planner.

## 8. What happens when something goes wrong

| Situation | Program response |
|---|---|
| Startup without a line | Zero output; waits for four valid, distinct frames |
| Normal line | Continuous steering with look-ahead, filtered derivative and command ramps |
| Curve | Reduces speed and increases left/right speed difference |
| Low confidence or missing line | Immediately commands zero; pauses for 0.20 s |
| Lost during a turn | If enabled, pivots toward last known direction, then scans the other side within a bounded time |
| Lost after a centred straight with no known direction | Stops; does not guess a blind forward route |
| Brief glimpse during recovery | Stops pivoting while verifying four valid new frames |
| Confirmed reacquisition | Resumes slowly through the ramp; derivative is reset |
| Not reacquired within 4 s of loss | Latches stopped and exits; reposition/fix and restart |
| More than three losses within 20 s | Latches stopped and exits |
| Large tracking error persists for 2 s | Stops rather than continuing to push off the line |
| Fork, broad crossbar or ambiguous marking | Stops; does not invent a route |
| No fresh host-received image within 0.25 s | Stops and exits |
| Main loop misses motor updates for 0.30 s | Software watchdog disables outputs and latches a fault |
| Camera disconnect, exception, Ctrl+C, SIGTERM, or `q` | Disables outputs during cleanup |

`BRAKE` is a state name: output zero removes drive; this code does **not** command active electrical braking. The robot can coast. Recovery rotates in place and has no obstacle sensing. Keep the test area clear and use a physical motor-power switch. A Python watchdog cannot guarantee shutdown if the OS hangs, the process is force-killed, or the GPIO hardware fails; use a hardware watchdog/enable circuit when that guarantee is needed.

Set `recovery.enabled` to `false` if you want every sustained line loss to end the run. There is no blind gap-crossing, obstacle avoidance, encoder-based stall detection or automatic branch choice.

## 9. Test and replay without motors

```bash
python3 -m unittest -v test_line_follower.py
python3 generate_demo.py
python3 line_follower.py --video synthetic_track.avi --show --log demo.csv
```

The generated video demonstrates acquisition, a right curve, temporary loss and recovery, a left curve, then a long loss and timeout. This is an image/state-machine exercise, not a physical dynamics simulation. A timeout exit code of 2 at the end is expected. Video replay is always dry-run; `--video --drive` is rejected.

For your own recording, supply its path to `--video`. Files play at their reported frame rate. The CSV includes state/reason, image error, bend, confidence, processing time, host-receipt frame age, estimated wheel speeds, actual requested PWM duty and curvature command. Existing log files of the same name are overwritten; choose new filenames for separate experiments.

The replacement uses the local `--show` window and CSV logs. Your earlier Flask browser monitor on port 5000 is not included in this package. Do not run the old and new motor scripts simultaneously: they control the same GPIO pins.

## References used

- [GPIO Zero output devices and BCM numbering](https://gpiozero.readthedocs.io/en/stable/api_output.html)
- [GPIO Zero pin backends, including lgpio](https://gpiozero.readthedocs.io/en/stable/api_pins.html)
- [Raspberry Pi hardware and GPIO](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
- [OpenCV capture properties and backend limitations](https://docs.opencv.org/4.13.0/d4/d15/group__videoio__flags__base.html)
- [Differential-drive kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html)

The tuning constants are project starting values, not values prescribed by these references. The source code has not been tested on your Raspberry Pi, webcam, driver or motors.
