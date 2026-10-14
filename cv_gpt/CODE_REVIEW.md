# Review of your two previous scripts

Reviewed 14 October 2026. Sources: the uploaded `Pasted text(5).txt` line follower (684 lines) and the motor-direction diagnostic pasted in chat. Original files were not modified. Line numbers below refer to the uploaded file unless stated otherwise.

The source contains several concrete control weaknesses that can explain good straight-line performance and poor curves. This does not establish the single physical root cause without a track video, detected mask and actual motor measurements.

## Applied to the replacement package

| Setting | Updated value |
|---|---|
| Left IN1 / IN2 / enable PWM | BCM 4 / 17 / 23 |
| Right IN3 / IN4 / enable PWM | BCM 27 / 22 / 24 |
| PWM frequency | 100 Hz, matching both supplied scripts |
| Positive left/right commands | IN1/IN3 HIGH; IN2/IN4 LOW, matching your code |
| Four motors | Two side channels: front/rear left together, front/rear right together, if wired that way |

Only the known wiring/frequency has been copied. The pin names do not prove the board is an L298N, and neither script provides wheel spacing, measured speed, wheel diameter or motor current. `hardware.confirmed` remains false and `wheel_track_m` remains null until the relevant checks/measurement are complete. No guessed measurement has been promoted to a fact.

The existing replacement already uses near/far path information, filtered time-based derivative, curve slowdown, zero output on loss, bounded recovery, acquisition checks, stale-frame checks and a command watchdog. These were retained. Source still uses GPIO Zero; the GPIO numbers do not require switching back to RPi.GPIO.

## Uploaded line-following script: priority findings

### 1. The minimum PWM weakens ordinary turns

Evidence: `normalize_speed()` at lines 119–125; `BASE_SPEED=26`, `MAX_CORRECTION=20`, `MIN_MOTOR_PWM=22`; mixing at line 614.

At a positive saturated correction, the controller requests left=46%, right=6%. Normalization changes this to **46% and 22%**. The right/inside wheels cannot slow to the requested value. Every nonzero input from 1% through 21% becomes 22%, creating a discontinuity between zero and the minimum.

Reproduced by executing the original pure normalization function with no GPIO access:

| Input PWM | Output PWM |
|---:|---:|
| 0 | 0 |
| 0.9 | 0 |
| 1 | 22 |
| 6 | 22 |
| 21 | 22 |
| 46 | 46 |

The existence of a motor deadband is real, but blindly raising the inside-wheel command reduces the requested left/right difference. Actual speed is not proportional to PWM, so an exact turn radius cannot be calculated from these duty values alone. The replacement uses measured-speed/duty tables when calibrated, permits the inside wheel to reach zero, and does not force a high minimum duty. Below the reliable moving range, an uncalibrated motor can still stall.

### 2. The code traces far rows but discards them for steering

Evidence: lines 291–340, especially `bottom = centers[:4]`.

When all scan rows contain tape, steering uses the four nearest rows (118, 112, 106, 100 in the 120-pixel image). Farther points affect tracing/diagnostics but not the final steering coordinate. The comment about bottom rows following a bend earlier is misleading: those rows emphasize the closest ground, so a bend first visible farther ahead cannot change this coordinate.

Reproduction: two masks with identical near-four rows at x=80, but other rows at x=80 versus x=130, both returned **cx=80.0**. The replacement includes farther path direction explicitly. Camera angle still determines whether the approaching bend is visible at all.

### 3. It drives after losing the line, including at startup

Evidence: lines 460–463 initialize both last commands to 26; lines 623–645 implement loss handling.

When detection disappears, the script repeats the previous command for up to **1.5 seconds**, then searches for up to **8 more seconds**. If the previous command was an aggressive pivot, that pivot continues without visual feedback. If it never saw a line at startup, it still begins with the initialized forward commands of 26/26. The fallback first-search direction is right when no direction is known and the stored centre is centred.

The replacement starts at zero, requires several valid frames, removes drive immediately on loss, and searches only after a known turn direction. Its four-second timeout includes the initial pause and recovery checks. A centre-line gap with no useful direction produces a stop rather than blind forward bridging.

### 4. A single row can count as successful detection/recovery

Evidence: `trace_line()` only rejects an empty `centers` list (lines 332–340); the main loop treats any non-None `cx` as found (line 573).

There is no minimum count of valid rows, mandatory near-ground coverage or confidence score. One isolated candidate can reset the lost timer and trigger motion. A mask with only four foreground pixels at x=20–23 on far row 46 returned **cx=21.5**. This is a direct test of `trace_line`; it does not claim those exact pixels necessarily survive the earlier morphology.

The replacement requires multiple bands, near-field coverage and confidence, then consecutive fresh-frame confirmations for initial acquisition/recovery. Those checks reduce false positives; they do not make detection infallible.

### 5. Wide merged regions still bias the steering coordinate

Evidence: lines 308–328 append `own_center` to the steering samples before the `merged` decision. The merged branch changes `reference` for prediction only; it does not remove the already-added centre.

The comment says wide merged regions should not redirect the line, but they still influence the returned centre. A broad 100-pixel region on the nearest four scan rows returned **cx=69.5** despite a prior centre of 80. `MAX_LINE_WIDTH=145` also admits a region spanning about 91% of the image.

The replacement stops on broad/ambiguous markings rather than choosing a route. This is conservative and may also stop on legitimate tight bends whose apparent tape width is large; inspect and tune the actual mask.

### 6. Steering jumps abruptly from a mild turn to 85% / -40%

Evidence: lines 46–50 and 595–614. Above a 22-pixel error, normal steering switches to a fixed reverse/forward command.

A right turn can jump from approximately **46/22** to **85/-40**. These are large step changes in PWM and direction, not a gradual adjustment. Under load, this can cause overshoot, tyre scrub or supply stress. The precise physical response depends on the drivetrain and battery.

The mirrored signs in `pivot_left()` and `pivot_right()` are internally consistent, assuming positive commands really drive each side forward. However, 85% on one side and reverse 40% on the other are not evidence of a centred pivot. They only become a centred pivot if the resulting opposing ground speeds happen to match. The separate diagnostic uses equal 40/40 commands, so it does not reproduce the full follower's asymmetric pivot.

The replacement uses gradual wheel commands and lower bounded recovery commands. Normal tracking does not reverse a wheel; a sharp corner may still require line-loss recovery or dedicated track-specific logic.

### 7. Direction changes occur before PWM is disabled

Evidence: lines 128–139.

`drive_one_motor()` switches direction pins while the previous PWM duty may still be active. Normal-to-pivot and alternating-search transitions can therefore reverse under drive. This may create a jerk/current transient; its electrical consequences depend on the H-bridge.

The replacement turns PWM off before changing direction, clears direction pins, and includes a short dead interval for a reversal. Command ramps further soften changes. This does not replace proper current limits, electrical design or mechanical testing.

### 8. The derivative and filtering depend on frame timing

Evidence: `derivative = error - previous_error` at line 591; fixed per-frame centre smoothing at line 582.

The derivative is change per frame, not change per second. It can be tuned for a fixed sampling rate, but its effect changes when processing/camera timing changes. The replacement divides by measured elapsed time and filters the derivative. Gains have different units and cannot be copied unchanged just by adding the division.

Your original does correctly reset `previous_error` on reacquisition, which avoids a large stale-error derivative jump in that situation.

### 9. Failed camera reads stop the motors; blocked reads do not

Evidence: main `cap.read()` at line 491 followed by error handling at lines 492–499.

If a read returns failure, the script stops immediately—this is good. If it blocks, the loop cannot reach the stop call and the last motor command remains active. There is no independent command timeout. A successful read can also be an old buffered image; requesting `CAP_PROP_BUFFERSIZE=1` alone does not prove fresh capture.

The replacement separates continuous capture from processing, checks host-receipt freshness, and uses a software command watchdog. It still cannot detect all sensor/driver buffering or guarantee shutdown after an OS hang/force-kill; those require hardware support.

### 10. Stopping after a search is not latched

Evidence: lines 646–649 zero the commands but keep the loop running. Any later `cx` at line 573 returns to tracking, even after a timeout, and there is no stable reacquisition requirement.

The replacement exits with outputs disabled after a terminal recovery fault. Reposition/fix the issue and restart to resume. It also limits repeated loss episodes within a time window.

### 11. Thresholding and low resolution need real-image tuning

Evidence: capture 320×240 is reduced to 160×120, a 5×5 blur is applied, then fixed threshold 65 and 3×3 opening/closing.

These are not inherently invalid choices. However, low-resolution distant tape may be only a few pixels wide; blur/opening can erase it, while closing can merge nearby features. A fixed threshold of 65 can miss tape under changed exposure or classify shadows as tape. `EXPECTED_LINE_WIDTH=12` also does not model perspective.

The replacement starts at 640×480 with configurable fixed/Otsu/adaptive thresholding, narrower filtering and explicit detection checks. Higher resolution costs processing time and is not automatically better on every Pi. Benchmark actual performance and inspect the mask before tuning gains.

### 12. JPEG streaming still consumes control-loop time

Evidence: lines 664–674 call `build_stream_jpeg()` in the control thread. The Flask server is a separate thread, but image preparation/encoding is not.

Limiting encoding to 10 FPS reduces work but does not remove the timing cost. This can interact with frame-dependent control. The replacement package currently has a local display and CSV logs, not the Flask browser stream. If browser streaming is added, use a latest-frame monitoring worker and measure its effect on control timing.

### 13. Turn memory can be stale; optional routes are time-based

Evidence: `last_turn_direction` is updated for errors beyond ±12 but is never cleared on a long straight. Loss may therefore search in a direction from an earlier bend. `search_direction()` starts with a 1.6-second leg (0.8×2), not a 0.8-second leg, then 2.4 and 3.2 seconds; this follows the code but deserves attention when tuning.

Intersection routing is **disabled by default**, so it is not currently responsible for route overrides. If enabled, routing uses fixed 0.45/0.75-second actions with no position/heading feedback; it may miss the intended branch. A five-frame clear condition can also unlatch an intersection while a turn is still active, allowing a fresh detection to restart its route timer. An empty route list would fail on indexing/modulo. These are conditional issues, not claims about your current disabled mode.

## Pasted motor-direction diagnostic

This is a useful test. Positive/negative commands are mapped consistently, tests are separated by zero output and a pause, and `finally` cleans up after normal completion or an exception inside the test block. I do not see an inherent left/right sign error in the source. Physical wiring determines whether positive really means forward.

Its limitations and issues are:

1. **It checks sides, not four independent motors.** With two motors on each channel, verify both wheels on that side rotate in the direction that would move the chassis forward. A crossed/reversed connection on one wheel can make the pair fight.
2. **Lifted-wheel success does not establish loaded turning ability.** It cannot validate friction, wheel scrub, required torque, combined current or battery sag.
3. **Setup is outside the try/finally.** Failure partway through GPIO/PWM construction can leave initialized resources without cleanup. Unlike the full follower, this diagnostic's `GPIO.setup(..., GPIO.OUT)` does not explicitly set the initial output LOW.
4. **No input bounds in `drive_motor()`.** Its current hard-coded ±40 commands are within range, so this is not presently breaking the tests. Reusing the function with values outside ±100 can fail; validate/clamp before writing GPIO.
5. **The helper itself does not disable PWM before a reversal.** The supplied test sequence avoids a direct powered reversal because it calls `stop_motors()` between tests. This is a reuse hazard, not evidence that this specific sequence reverses under power.
6. **No direction-invert option per side.** If one side is reversed physically, correct wiring or use an explicit inversion setting rather than changing steering signs throughout the follower.
7. **No periodic command watchdog during the sleep.** A normal Ctrl+C reaches finally, but the script does not provide an independent shutdown guarantee if execution freezes. The replacement includes a software watchdog, with its documented limits.
8. **Zero output is not verified physical braking.** With PWM disabled and direction pins LOW, the actual driver may coast. Neither script should assume the chassis stops instantly.

## What to check first on the robot

1. Confirm each two-wheel side moves forward for a positive command and that the sides are not swapped.
2. In dry-run, put the robot just before a curve: verify farther points change the replacement's error before the nearest tape moves out of centre.
3. Check the raw image and mask together. If the camera still clearly sees tape but the mask loses it, tune segmentation rather than replacing the camera.
4. Measure wheel-centre spacing and loaded motor response. The replacement's sample 0.10 m/s maps to about 42% duty, while the old base is 26%; matching GPIO does not match speed.
5. Run slowly on a clear track and examine the requested inside-wheel command, actual wheel movement, and recovery state. Increase speed only after both turn directions are reliable.

The strongest source-level explanation is weak ordinary steering followed by late, aggressive pivots, compounded by continuing the previous command after losing the line. Camera limitations and four-wheel mechanics remain possible contributing causes.
