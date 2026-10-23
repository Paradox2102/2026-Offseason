# Shooting changes and a calibration tool: design

Date: 2026-10-22. Branch: `nick-test`. Approved by the user (2026-10-22). Builds on
`2026-10-22-nick-test-rewrite-design.md` (feed path, trench rules, rack trust, shoot on the move,
double trench auto).

Two sub-projects, built and pushed in this order:

1. **Shooting changes:** clear the turret entrance after a shot; pack and pulse the intake while
   shooting.
2. **Calibration tool:** procedures in Test mode that measure the values the robot must give, save
   them in a file on the robot and use them over the constants.

---

## Sub-project 1: shooting changes

### 1.1 Clear the turret entrance after a shot

Today every feed stop is the same: the vertical roller (Talon 40, the last stage) runs on 0.25 s,
then every stage brakes. A ball can stay at the turret entrance after the shot.

Two kinds of stop now (`FeedLogic`, pure and tested):

- **Pause:** a short "not ready" in the middle of a shot (`Mode.STOP`). As today: the vertical
  roller runs on 0.25 s, then every stage brakes.
- **End:** the trigger is released, or the auto's shoot command ends (`Mode.END`). The vertical
  roller runs on 0.25 s, THEN every stage runs backwards for 0.5 s
  (`ShooterConstants.kFeedClearTime`), then every stage brakes. The backwards speed is the eject
  speed (half the feed speed).
- An end that comes after a pause starts the reverse at once (the run-on is over). An end in the
  middle of an unjam goes on backwards (the clear time from the end).
- The reverse runs only if a stage fed since the last end: a shot that never got ready does not
  move the balls.
- `Feeder.feedWhen` gives `STOP` while not ready and `END` when its command ends. `Feeder`
  keeps applying the logic while no command runs, so the reverse finishes after the release.
  `isFeeding()` is false while the feed clears. The dashboard state reads "Clearing".

### 1.2 Pack and pulse the intake while shooting

User: while shooting, bring the intake in a little to push the balls in the hopper toward the
feed, and pulse it; not so far in that it blocks the shots. The real distance is found on the
robot.

- **Timing** (`superstructure/PackPulse`, pure and tested; time since the feed first started in
  this shot):
  - before 1.0 s (`kPackDelay`): no pack, the intake stays fully out;
  - from 1.0 s: the rack goal is the pack position;
  - from 1.0 s + 0.3 s (`kPackSettleTime`, "once there"): the goal is the pack position plus a
    sine of the pulse amplitude at the pulse rate, starting at 0.
- **Pack position** = a tunable fraction of each side's out stop (start 0.85: 0.275 m, 4.9 cm in).
  **Pulse** amplitude 2 cm and rate 4 Hz, tunable. Dashboard tunables under `Intake/Pack/`.
- The goal of each side is clamped to 1 cm inside each stop: a position move never pushes a stop.
- While packing, the intake rollers run in (`IntakeRoller.runInWhile`): they run in from the
  roller's own periodic only when no command drives the rollers, so the driver's intake and the
  auto's intake event keep working.
- **Shot ends** (trigger released, auto shoot ends): no pack; the rack goes fully out with the
  normal stop logic (Motion Magic, push, stall, re-true), so `isExtended()` is true again only once
  each side is at its out stop.
- **Near a trench:** no pack, no pulse, the intake stays fully out. Teleop and auto alike (the
  auto's "shoot" event uses `Superstructure.shoot()`). "Near" for the pack is wider than the trench
  guard's (`TrenchGuard.nearTrench(pose, velocity, lookahead, extraMargin)`): 0.6 s more look ahead
  and 0.8 m more room, because the rack needs ~0.4 s to get fully out again, and the double trench
  auto stops in the lane, then goes at 3.5 m/s (a velocity look ahead does not see that coming).
  Found in the sim: with the guard's own "near", the rack was still going out when the auto's trench
  brake checked, and the auto gave up the second trench.

**Rack position mode.** `IntakeRack` is stop-based (`RackSide`): every move ends with a push into
a hard stop, and short stalls are never trusted. A new phase `POSITION` holds a Motion Magic goal
and has no push and no stall logic. It never sets the Talon position and never marks a side as
confirmed, so it cannot weaken the trust rules. A side leaves `POSITION` only through a normal
move to a stop (`start(out)`), which confirms it there again.

- `IntakeRack.packWhile(Supplier<Pack>)`: the rack asks each loop; `null` = no pack. It packs only
  while enabled with the goal out (the shot holds the rack out). A rebooted side resyncs first.
- `isExtended()` stays strict: false while packing (whatever the positions read).
- `extend()` and `retract()` stop waiting while the rack packs (it is as far out as allowed).
- `isPacking()` and `atLeast(fraction, tolerance)` for the shot gate.

**Shot gate** (`Superstructure.allowed()`): while the rack packs, the intake check is "each side
at least at the pack position minus the pulse amplitude minus 3 cm"; else `isExtended()`. The
dead-Talon rule is unchanged (`|| hasDeadTalon()`). (3 cm, not 1.5: in the sim the rack cannot
follow a 2 cm, 4 Hz sine, which needs 12.6 m/s^2 against its 6 m/s^2 profile; it lags and swings
up to 1.5 cm further.)

**Rack kA.** The rack had no acceleration feedforward; with only kP 30 V/m it lagged a moving goal
and overshot 2.7 cm on the 5 cm pack move. `IntakeConstants.kRackA` from the physics (side mass on
the pinion, through the gears, as X44 current, as volts: ~0.1 V per m/s^2) cut that to 1.6 cm.

**Auto stuck check.** The double trench auto gives up the trench when the trench brake holds with
the intake not out (after 2 s). That must now last 0.5 s (`kStuckTime`): a rack going back out
after a pack takes a moment.

### 1.3 Smoke test

- Teleop shot step (13): print the pack (time from the first feed to the pack, the lowest and
  highest rack position while packing, the shot gate kept open) and, after the release (step 15),
  the clear (vertical roller run-on, then the time all stages ran backwards).
- All autos must still pass: right, left, right-sotm, left-sotm, red, and `SIM_SMOKE_RACK=blocked`.

### 1.4 Sim results

143 unit tests pass. Smoke runs (all PASS): right, left, right-sotm, left-sotm, red, red-sotm,
`SIM_SMOKE_RACK=blocked`, `SIM_SMOKE_RACK=dead`.

- **Clear:** after the release the vertical roller ran on 0.26 s, then every stage ran back from
  0.26-0.28 s for 0.48-0.50 s (vertical roller -45 rps), in every run.
- **Pack (teleop shot):** the pack starts 1.01-1.02 s after the first feed; the feed kept going the
  whole time it packed (0.52-0.60 s, to the end of the step). Pulsing, the rack read 0.242-0.313 m
  (pack position 0.275 m): it lags the 4 Hz sine and swings past it. Gate at 0.225 m.
- **Autos:** right 45 of 45 scored, left 44 of 44, red 43 of 43: no contacts, intake 100 % out at
  the trench. Right/left/red shoot on the move: 41-42 of 41-42, 2.8-3.2 s of feeding on the move,
  no fallback, no contacts. Their trench brake held 12-14 loops: the same as before this change
  (13 loops at c0bcb0f); it waits for the hood, which the shot leaves up until the trench guard
  stows it. Blocked: fallback, no shot. Dead Talon 22: fallback, 8 of 8 preload scored.
- In the autos the robot packs only while it shoots away from the trench lanes: the double trench
  auto shoots in the lane and then drives under the arm, so there it does not pack.

### 1.5 Hood down before the trench (user, 2026-10-22)

The double trench auto's first path ends stopped in the trench lane at (3.1, 0.7), and path 2
goes from there straight under the arm. The "shoot" event ran until the trench guard said "near",
which came only as path 2 started, so the hood was still up and the trench brake held the robot
~0.25 s (12-15 loops). Now the routine ends the shot when path 1 ends: the hood comes down while
the robot is stopped. The smoke test prints when the last ball is shot and scored (auto ends at
20.0 s).

Sim (trench brake loops; last ball shot / scored; balls scored):

| Run | Brake | Last shot | Last scored | Scored |
|---|---|---|---|---|
| right-sotm | 2 (was 13; the 2 are the start, intake going out) | 16.2 s | 17.1 s | 40 of 40 |
| left-sotm | 0 (was 12) | 16.6 s | 17.5 s | 41 of 41 |
| right-sotm, red | 1 (was 14) | 16.1 s | 17.0 s | 41 of 41 |

The last shot (after path 2) starts at 14.4-14.5 s (was 14.7-14.8). Every ball is scored 2.5 s or
more before the auto ends, with none left in the hopper.

---

## Sub-project 2: calibration tool

### 2.1 Store

- **File:** on the SystemCore `/home/systemcore/calibration.json` (the robot user's home, next to
  `logs/`); in the sim `build/calibration.json`. JSON (Jackson, already on the classpath through
  WPILib).
- **Load at boot,** before the subsystems are built: each value that is in the file is used over
  its constant. A missing or unreadable file: constants only, and a MEDIUM alert for a bad file.
- **Save:** the old file is copied first to `calibration-backup/calibration-<time>.json` next to
  it; the new file is written to a temporary file and moved over the old one. The values are
  printed to the console as Java constant lines, so they can be committed later; the dashboard
  shows where it saved (`Calibration/Message`).
- `calibration/CalibrationData` (a record of optional values) and `calibration/CalibrationStore`
  (load, save, backup). Unit test: save and load round trip, the backup, a missing file, a broken
  file. The JSON is read and written by a small reader in the store: WPILib 2027 brings no JSON
  library, and BLine's json-simple is not ours to depend on.
- `CALIBRATION_FILE` (environment) picks another file. The sim scripts use their own: the smoke
  and shoot-on-the-move runs a file that does not exist (constants only), the calibration run
  `build/sim-calibration.json`.

Stored values (each optional: missing = the constant):

| Value | Used | Replaces |
|---|---|---|
| Shot points: distance, flywheel wheel rps, hood deg, time of flight s (optional) | at once | `ShooterConstants.kScore*` tables near each point |
| Steer offsets (4, rotations) | next code restart | `TunerConstants.k*EncoderOffset` |
| Turret trim, degrees | at once | 0 |
| Intake out stops (left, right), meters | at once | `IntakeConstants.kOutStopLeft/Right` |
| Hood ratio; hood stop angle above level, degrees | at once (Talon config applied again) | `kHoodRatio`, `kHoodStopAboveLevelRot` |
| Feed speeds (5 stages), motor rps | at once | `ShooterConstants.k*Rps` |

### 2.2 Operation

- Elastic tab **Calibration** (`tools/make-elastic-layout.py`): the procedure chooser, the step,
  its instructions, live readings, the typed values, Capture and Save buttons.
- **Test mode only.** WPILib 2027 calls Test mode "Utility" (`RobotState.isUtility()`). The test
  gamepad (port 2) acts only in Utility mode and only for the procedure picked in the chooser. The
  dashboard buttons do nothing outside it (the message says so).
- The SysId bindings of the test pad (`RobotContainer.configureTest`) become the procedure
  "Drive SysId", so no buttons clash.
- Each procedure is a class with the same actions the buttons call, so the sim script calls them
  directly (headless gamepad buttons do not reach the robot code in the sim).

### 2.3 Procedures

1. **Shot table.** The robot at a spot. Distance: from the pose (vision), or typed
   (`Calibration/Shot/Typed Distance m`, 0 = the pose). Test pad: D-pad up/down jogs the flywheel
   (1 rps, bumper held 5), left/right the hood (0.5 deg, bumper 2), RT shoots (the turret aims by
   itself; the feed waits for the jogged speed and hood), A records the point (with the typed
   time of flight if not 0), B deletes the recorded point nearest the distance, Y saves.
   - A point recorded within 0.1 m of a recorded point replaces it.
   - `calibration/ShotPoints` (pure, tested): the recorded points merged into the constant table:
     a recorded point replaces every constant point within 0.25 m of it. `ShotCalculator` takes
     the new tables at once.
2. **Steer zero.** Line every wheel up straight forward with a straight edge (bevel gears the way
   Tuner X had them), press Capture (or A). New offset = old offset - absolute reading, wrapped to
   +-0.5 rotation (`calibration/SteerOffsets`, pure, tested). The swerve is built at boot, so the
   offsets apply at the next robot code restart: the step says so.
3. **Turret zero.** A (or the dashboard) runs the jig zero (`Turret.zeroInJig`). Optional trim: jog
   the turret (D-pad left/right, 0.25 deg, bumper 2) until the shooter lines up with a mark at the
   jig angle; X records the trim (the reading there). `Turret` reads `position - trim` and aims at
   `goal + trim`, so the trim is used at once.
4. **Intake zero.** A starts it: three times in and out, both sides; each side keeps its largest
   stall reading (`Intake/Rack/<side>/Last Stall m`, the reading before any fix). The new out
   stops replace the constants at once (`IntakeRack.setOutStops`).
5. **Hood zero.** A homes the hood to its lower stop. D-pad up (held) drives it up slowly (1 V,
   past the soft limit) to the upper stop; the rotor turns are read. Type the measured angle from
   the lower to the upper stop: ratio = rotor turns / (angle / 360). Type the angle above level at
   the lower stop (for kG later). X computes; the Talon config is applied again and the hood homes
   again.
6. **Feeder tuning.** D-pad left/right picks a stage, up/down jogs its speed (5 rps, bumper 20),
   X (held) runs that stage alone (its speed and current show). RT shoots normally; the fire-rate
   test counts flywheel speed dips (`calibration/DipCounter`, pure, tested: a dip = the speed
   falls under 95 % of the goal and comes back over 98 %) and gives balls per second. The new
   speeds are used at once.

Y saves in every procedure (and the dashboard "Save").

### 2.4 Tests

- Unit: store round trip and backup, shot point edits and merge, steer offset math, dip counter.
- Sim: `tools/sim-calibration.sh` (`sim/SimCalibrationTest`, `SIM_CALIB=1`) runs the procedures in
  Test mode, records a shot point and checks that the shot the robot uses changes to it, saves,
  and reloads the file.
- `tools/sim-smoke.sh` still passes, with the Calibration tab in the layout check.

### 2.5 Sim results

162 unit tests pass. `tools/sim-calibration.sh` PASS:

- Outside Utility mode: Save refused.
- Shot table at 2.44 m: table 85.2 rps; jogged to 91.2 rps and hood 20.5 deg (+6 rps, +2 deg),
  8 balls shot (none scored: the jog was on purpose too far). Recorded: the table gives 91.2 rps
  and 20.5 deg there, the robot's own shot the next loop is 91.1 rps, the 5.5 m shot is unchanged.
- Steer zero, wheels forward: new offsets within 0.09 deg of those in use (the sim has no offset
  error to find).
- Turret: turned to 3 deg, trim 2.7-3.2 deg recorded, the turret then reads 0.00.
- Intake zero, 3 cycles in 12 s: out stops 0.3240 m (left) and 0.3298 m (right): the sim's stops
  are 0.324 and 0.330 (the right one 6 mm past the constant on purpose).
- Hood zero: up to the sim's upper stop (41.9 deg, 9.90 motor turns), typed 42 deg: ratio 84.85
  (the sim's 85).
- Feeder: the indexer speed jogged 96 -> 101 rps, used at once. Fire-rate test: 0 dips: the sim
  flywheel has no ball load, so the dip counter is covered by `DipCounterTest` only.
- Saved; the file reads back the same. A second boot with that file: every value in use (shot
  point in the table, steer offsets, turret trim, out stops, hood ratio, feed speeds).
- The intake zero makes CTRE print "Warning 10002: Do not apply or refresh configs periodically":
  each stop re-trues the rack with `setPosition` (a config call), 7 times in 10 s. Harmless; the
  script ignores that warning's stack trace.

Smoke (all PASS, run with no calibration file): right 44 of 44, left 44 of 44, red 43 of 43,
right/left/red shoot on the move 41-42 of 41-42 (trench brake 11-15 loops, as before), blocked and
dead as before; the layout check finds every widget topic, the Calibration tab's too.
`tools/sim-sotm.sh`: every ball shot scored up to 3 m/s (mean miss 0.05-0.13 m, 0.25 m on a 3 m/s
arc, 0.34 m strafing at 2 m/s while spinning at 2 rad/s), as before.

### 2.6 To measure on the robot

Calibration tool (Utility mode, Calibration tab):

- Shot table at 1.5-6 m (and the time of flight, from 240 fps video), then the pass tables.
- Steer zero with a straight edge; restart the code; check with "Pit/Wheels Forward".
- Turret: check the jig angle against a mark; trim if it is off.
- Intake zero (out stops), with nothing in the way.
- Hood zero: the ratio (now a CAD estimate of 85) and the angle above level at the lower stop;
  then give `kHoodG` a value.
- Feeder tuning: the stage speeds and the fire rate with a full hopper.

Shooting changes:

- Pack fraction (start 0.85): the most the intake can come in without blocking a shot, and enough
  to push the last balls. Pulse amplitude and rate (2 cm, 4 Hz): the rack's 6 m/s^2 profile cannot
  follow 2 cm at 4 Hz; watch "Shooter/Blocked By" for "intake not out" during a pulse.
- The clear (0.5 s back): no ball left at the turret entrance, none pushed out of the hopper.
- Rack kA (~0.1 V per m/s^2, from the physics): check with SysId or a step response.
- The double trench auto waits ~0.25 s at the trench for the hood to come down after its shot
  (before this change too): ending that shot a little sooner would save it.
