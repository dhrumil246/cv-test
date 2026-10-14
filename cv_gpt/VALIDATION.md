# Validation record

Created 14 October 2026.

Updated the same day after reviewing the supplied scripts: left BCM 4/17/23, right BCM 27/22/24, PWM 100 Hz. Re-ran all 32 tests successfully with this configuration, including the mocked GPIO direction/standby/stop test. Physical wiring and loaded motor behaviour remain untested.

The original script's pure functions were extracted with Python AST and executed without importing its GPIO/camera/server startup code. Checks reproduced (a) 46/6 requested PWM becoming 46/22, (b) identical returned steering centres despite a changed far path when the nearest four samples were unchanged, (c) acceptance of a single far-row segment, and (d) a wide merged region influencing the returned centre. See CODE_REVIEW.md for exact results and the separate diagnostic-script review.

## Completed

- Python syntax compilation succeeded.
- 32 automated tests passed using Python 3.12, OpenCV 5.0.0, NumPy 2.5.4 and GPIO Zero 2.0.1.post3 in the development environment.
- Vision tests covered centred/offset tape, left/right curves, black/white tape, fixed/Otsu/adaptive thresholds, an illumination gradient, blank images, missing near-field tape, broad crossings and parallel branches.
- Controller tests covered distinct-frame acquisition, steering sign, curve slowdown, forward-only tracking limits, immediate stop on loss, recovery sweeps, minimum stop time, stable reacquisition, bounded recovery, repeated losses, persistent error and stale-frame faults.
- Motor tests covered PWM conversion, output limits, software-watchdog latching, and direction/standby/stop operations through GPIO Zero's MockFactory. No physical GPIO was accessed.
- Full CLI replay on the generated synthetic video reached WAIT_LINE → TRACK → BRAKE → SEARCH → TRACK → BRAKE → SEARCH → STOPPED. Temporary loss recovered; the final long loss ended with `line_recovery_timeout` and zero left/right PWM. Exit code 2 is expected for this stopped state.
- The CLI rejected `--video --drive` before motor initialization.

## Not established by these tests

- Physical turning radius, traction, loaded motor speed, stall current, braking distance or robustness to battery changes.
- Raspberry Pi performance, USB camera latency/control support, real GPIO PWM timing or the actual driver's electrical behaviour.
- Reliable handling of every shadow, reflective floor, branch, gap, finish marker or right-angle corner.
- Hardware-level shutdown after process force-kill, OS failure or power faults.

The included PWM calibration tables are placeholders. The 0.16 m preview wheel spacing is an explicitly unmeasured example; actual drive mode requires a measured value. Host frame-receipt age does not measure sensor-to-host buffering. Synthetic replay tests the program and images, not closed-loop physical driving.

Re-run on the Pi with `python3 -m unittest -v test_line_follower.py`, then perform the dry-run, direction and low-speed track checks in README.md. OpenCV APIs used here are also available in the 4.x series, but the Raspberry Pi's installed software/hardware combination has not been exercised here.
