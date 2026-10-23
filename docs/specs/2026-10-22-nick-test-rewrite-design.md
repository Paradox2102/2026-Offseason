# nick-test rewrite: overall design

Date: 2026-10-22. Branch: `nick-test` (from `main` 29cc0c1).

## Goal

Make the 2026 Offseason robot code as good as the hardware allows. Use Motion Magic, Phoenix Pro
features, SystemCore, PhotonVision, BLine, simulation, logging and Elastic dashboards.

The students own the code on `main`. This branch is a reference: each change must be easy to read
and to copy back, one mechanism at a time.

## Constraints

- **Hardware.** SystemCore controller, 5 classic CAN buses at 1 Mbit/s (`CAN_S0` to `CAN_S4`).
  No CANivore, no CAN FD. So no signal time sync, no Dynamic Motion Magic, no 250 Hz odometry,
  no full `DifferentialMechanism`.
- **Licenses.** Phoenix Pro on every CTRE device (applied after CAEL; check each device, issue #8).
- **Software.** WPILib 2027.0.0-alpha-7, Phoenix 6 26.70.0-alpha-2, PhotonLib 2027 alpha-2,
  BLine-Lib 2027.0.0-beta.3. No AdvantageKit and no maple-sim for 2027.
- **Mechanisms built, wired and safe to run (user, 2026-10-22):** swerve (1 to 12, Pigeon 0,
  `CAN_S2`), intake roller 20 and rack 21/22 (`CAN_S4`), indexer roller 30 and conveyor 31/32
  (`CAN_S1`), kicker 40, hood 41 (X44), flywheel 42/43, turret 44, up-kicker 45 (`CAN_S3`).
  PDH 21 alone on `CAN_S0`. No LED strip on the robot now.
- **Values that only the robot can give** (gains, camera poses, turret zero, soft limits) stay
  as named constants with a comment that says how to measure them. The code never guesses
  silently.

## Sub-projects

Each sub-project gets its own plan, is built and checked in simulation, and is pushed to
`nick-test` when it is done.

1. **Foundation and drive** (this document, in detail).
2. **Intake:** rack Motion Magic on both sides with a skew guard, homing, roller.
3. **Shooter suite:** turret aim, hood, flywheel, kickers, indexer, shot calculator, shoot on the
   move, superstructure state machine.
4. **Autos:** BLine paths with event triggers, auto chooser, pose reset and alliance flip.

## Architecture (all sub-projects)

### Package layout

```
frc/robot/
  Main.java, Robot.java          TimedRobot: logging start, scheduler, sim hooks
  RobotContainer.java            builds subsystems, binds controls, autos
  BuildConstants / RobotMode     sim vs real, tuning mode flag
  lib/                           code with no robot facts in it
    phoenix/PhoenixUtil.java     config apply with retry, signal refresh helpers
    sim/                         small physics helpers that feed TalonFX sim states
    util/                        AllianceFlip, geometry helpers
  constants/
    CanIds.java                  every CAN ID and bus in one place
    FieldConstants.java          hub, zones, field size
  subsystems/
    drive/   Drive, DriveConstants, TunerConstants (generated), DriveSim, SwerveTelemetry
    vision/  Vision, VisionCamera, VisionConstants
    intake/  (sub-project 2)
    shooter/ (sub-project 3)
    superstructure/ (sub-project 3)
  commands/  DriveCommands, characterization commands
  health/    HealthMonitor, SwerveHealth
  autos/     (sub-project 4)
```

### Hardware and simulation

Each mechanism uses the real `TalonFX` object on the robot and in sim. In sim, a small physics
class feeds the Talon sim state (`TalonFXSimState`) from a WPILib model (flywheel, arm,
elevator). The swerve uses the CTRE sim thread.

Reason: Phoenix runs the same firmware logic in sim, so Motion Magic, FOC and the configs are
tested with no second code path. An AdvantageKit-style IO interface per mechanism would double
the code and give no replay benefit without AdvantageKit. Hoot replay (Pro) replays the CTRE
signals from a match log.

### Logging

- **CTRE signals:** `SignalLogger` writes hoot logs for every CTRE device (all signals with Pro).
- **Robot code values:** `Telemetry.log("<Subsystem>/<Name>", value)`. One prefix per
  subsystem. Units in the name when not obvious (`Drive/Speed mps`).
- **Poses and module states:** struct topics, so AdvantageScope and Elastic show them.
- **Alerts:** `org.wpilib.util.Alert`, in groups that Elastic shows.

### Dashboard

Elastic loads `src/main/deploy/elastic-layout.json` from the robot. Tabs: Driver, Pit, Drive,
Vision, Tuning (later: Shooter, Intake). The Driver tab shows only what a driver needs in a
match.

### Control choices (summary, details per sub-project)

| Mechanism | Request | Reason |
|---|---|---|
| Swerve steer | `MotionMagicExpo` with kS, kV | smooth, fast turn with a plan the PID only follows |
| Swerve drive | closed-loop `Velocity`, `TorqueCurrentFOC` output | same feel at any battery voltage; slip current limits traction |
| Intake rack | `MotionMagicVoltage` position, each side its own loop | sides have no shaft between them |
| Turret | `MotionMagicExpoTorqueCurrentFOC` + robot-yaw-rate feedforward | holds the hub while the robot turns |
| Hood | `MotionMagicExpoTorqueCurrentFOC`, `Arm_Cosine` kG | gravity and load independent |
| Flywheel | `VelocityTorqueCurrentFOC` | fast recovery after each shot |
| Rollers, kickers, conveyor | `MotionMagicVelocityVoltage` | smooth ramp, battery independent |

---

## Sub-project 1: foundation and drive

### 1.1 Repo hygiene

- Stop tracking `build/`, `.gradle/` and `bin/` (the `.gitignore` already lists `build/` and
  `.gradle/`; the files were added before it). A Mac or Windows build then no longer changes
  tracked files.
- Delete dead code: `*.java.xxx`, `PathPlanningThing/DrivePath`, the old `PathPlanningThing/Autos`
  JSON, `Telemetry.java.xxx`, `utils/DoubleNumber`, `utils/ParadoxTalon` (replaced in 1.3).
- Keep `Issues2.md` and `Issues3.md` (review history).

### 1.2 Constants

`Constants.java` (one 430-line class) is split. CAN IDs and buses go to `constants/CanIds.java`.
Field values go to `constants/FieldConstants.java`. Mechanism values move into the mechanism's
package when that mechanism is rebuilt. Until then, the old classes stay, so the robot still
builds after each step.

### 1.3 Phoenix helpers

`lib/phoenix/PhoenixUtil`:
- `applyConfig(TalonFX, TalonFXConfiguration)`: apply with up to 5 tries, raise an Alert on
  failure (a config that did not apply is a silent, dangerous fault).
- `registerSignals(CANBus, BaseStatusSignal...)` and `refreshAll()` once per loop per bus, so
  each loop reads all signals in one batch.
- Every Talon is registered with `HealthMonitor` (as `ParadoxTalon` does now).

### 1.4 Robot loop and logging

- WPILib 2027 sends `Telemetry.log` to NetworkTables only (`/Telemetry/...`) and writes no file.
  `Robot` starts `DataLogManager` (it records every NetworkTables change to a `.wpilog` in
  `/home/systemcore/logs`, or `/u/logs` with a USB drive) and `DriverStation.startDataLog`.
- `Robot` starts `SignalLogger` (hoot) on the robot, not in sim unless a flag is set.
- `Robot` starts `WebServer` on port 5800 over the deploy directory, so Elastic can download
  its layout.
- WPILib 2027 does not publish Alerts to NetworkTables. `lib/util/AlertPublisher` keeps the
  robot's own `Alert` objects and publishes `errors`, `warnings`, `infos` arrays per group under
  `/Telemetry/Alerts/<group>` (the format of Elastic's Alerts widget).
- Match time goes to `/Telemetry/Match Time` (WPILib publishes no FMS table).
- One `Telemetry` naming scheme. Old names (`GyroAngle`, `FrontLeftModulePosition`, ...) go.
- Field: a table `/Telemetry/Field` with type `Field2d`, `Robot` as one `Pose2d` (Elastic reads
  only a single pose for the robot; WPILib's `Field2d` sends an array), and arrays for vision.
- Swerve for Elastic: `/Telemetry/Swerve Drive` with type `SwerveDrive` and the nine number
  topics that widget reads (`Front Left Angle`, ... `Robot Angle`, radians).
- Publish `Drive/Pose`, `Drive/Module States`, `Drive/Module Targets`, `Drive/Chassis Speeds`,
  `Vision/<camera>/Accepted Pose`, `Vision/<camera>/Rejected Pose`, `Vision/Tags`.
- Component poses for the AdvantageScope 3D model (`Final Poses`: turret, hood, intake) built
  from the mechanism positions (stubs at zero until sub-projects 2 and 3).

### 1.5 Health

Port `SwerveHealth` from `feat/swerve-health` (a2914a6), which merged into `feat/health-monitor`
and never reached `main`. Keep `HealthMonitor`.

### 1.6 Drive

- **Steer:** `SteerRequestType.MotionMagicExpo` on every request (Phoenix 26.70 defaults every
  request to `Position`). Steer gains kS 0.1 V, kV 0.68 V per module rps (0.12 x steer ratio
  5.68), kP 20, kD 0.1 (kP was 35 with no feedforward). The module itself writes the Expo profile
  (`MotionMagicExpo_kV = 0.12 x ratio`, `kA = 1.2 / ratio`). SysId values replace these.
- **Drive:** `DriveRequestType.Velocity` in teleop and autos. Output type `TorqueCurrentFOC`:
  the module then sends `VelocityTorqueCurrentFOC`, gains in amps per rotor rps. Start values:
  kP 3 A/rps, kS 2 A, kV 0 (from the X60 FOC torque constant 0.0194 Nm/A, ratio 7.36, wheel
  radius 0.055 m and about 60 kg: 0.5 m/s of error gives about 5 m/s^2). SysId replaces them.
  The slip current (110 A) limits the torque, which is traction control. A tunable `Drive/Open loop teleop` switch keeps the old open-loop voltage drive
  for the first robot test.
- **Teleop input:** joystick deadband first, then the curve (squared for translation, cubed for
  rotation), then scale. This fixes the "curve before deadband" issue from the review.
- **Heading hold:** when the rotation stick is in the deadband and the robot moves, the drive
  holds the last heading with `FieldCentricFacingAngle`. The gyro drift and the swerve skew no
  longer turn the robot.
- **Snap and aim modes:** buttons for snap to 0/90/180/270 degrees and "face the hub" while
  driving (uses the same heading controller).
- **Pose reset:** `resetPose` at the start of an auto and from a dashboard button; the bad
  `getState().Pose = ...` lines go (they change a copy). The pose getter that BLine uses reads
  `getState().Pose` live, because BLine reads the pose right after its reset and a pose cached
  in `periodic()` is one loop old.
- **BLine gains:** separate PID objects for BLine (BLine changes them: continuous input,
  tolerance, reset). Start at the BLine docs values: translation P 2.0, rotation P 1.0,
  cross-track P 0.2 (the old cross-track 6.0 makes the robot snake). BLine telemetry goes to
  `Auto/BLine/...`.
- **X-lock:** `SwerveDriveBrake` while a button is held.
- **Characterization:** SysId for translation, steer and rotation, and the wheel-radius test,
  each as a dashboard button and on a test controller (port 2). Results print and go to the log.
- **Odometry:** 100 Hz (CTRE default on CAN 2.0). `HealthMonitor` logs bus utilization; raise
  only after a log shows room.

### 1.7 Vision

Rewrite `Vision.java`:
- One `VisionCamera` per camera. Each loop: read all unread results, run the pose estimator
  (multi-tag on the coprocessor, lowest-ambiguity single tag as the fallback), filter, then add
  to the drive estimator.
- **Fix:** the current code never calls the estimator, so it never adds a measurement.
- **Filter:** reject a pose off the field, more than 0.25 m from the floor, with ambiguity above
  0.2 (single tag), or with a single tag farther than 4 m.
- **Trust:** standard deviations scale with distance squared and drop with tag count:
  `xy = 0.02 * d^2 / n`, `theta = 0.06 * d^2 / n` (single tag: theta not trusted).
- **Timestamps:** use the result timestamp, converted to the drive's time base as the Phoenix API
  needs.
- **Every unread result counts:** each result in the batch gives its own measurement with its
  own standard deviations, in time order. Nothing is kept from one loop to the next, so a frame
  is never added twice.
- **Estimator calls (PhotonLib 2027):** `estimateCoprocMultiTagPose(result)`, else
  `estimateLowestAmbiguityPose(result)`. `update()` no longer exists.
- **Field layout:** one `Field` for the estimator, the sim tags, the sim cameras and the BLine
  flip size (`FlippingUtil.fieldSizeX/Y`). Default AndyMark; a constant selects WELDED.
- Camera transforms: four cameras, from the 2026-10-19 CAD (user: all four are on the robot).
  Position = lens front center; x/y from the robot center, z from the bottom of the wheels.
  Roll assumes the camera part's +z is the top of the image (check each stream for a level
  horizon).

  | PhotonVision name | CAD part | x, y, z (in) | roll, pitch, yaw (deg) |
  |---|---|---|---|
  | front_right | cambrea 1 | -12.25, +9.60, 9.18 | -17.3, -30.0, -148.0 |
  | front_left | cambrea 2 | -12.19, -8.62, 16.63 | -2.1, -25.0, +148.0 |
  | side_left | cambrea 3 | -11.05, -13.54, 16.60 | -11.1, -25.0, -65.0 |
  | side_right (name to confirm) | cambrea 4 | -0.44, +13.78, 9.62 | -8.9, -30.0, +65.0 |

  The old values had roll and pitch swapped, wrong yaw signs on front_right and side_left, and
  heights near 12 in.
- Sim: the vision sim needs a true pose that vision does not change. `DriveSim` keeps an
  odometry-only pose from the module positions and the raw gyro (the CTRE sim has no wheel slip,
  so this is the truth). A Tuning button adds a pose error to the estimator, so the sim shows
  vision pulling the pose back. The cameras use the same `Field` as the tags.

### 1.8 Elastic

`src/main/deploy/elastic-layout.json` (Elastic 2027.0.0-alpha9 format: `version`, `grid_size`
128, `tabs[].grid_layout.containers[]` with `type`, `x`, `y`, `width`, `height`, `properties`).
Field widget: game "Rebuilt", `coordinate_system` "wall_blue" (the tag layout and the paths
have the origin at the blue wall corner). Tabs:
- **Driver:** field with robot pose, match time, auto chooser, alerts, "vision has target".
- **Pit:** health alerts, swerve self-test and wheels-forward buttons, CAN bus use, battery.
- **Drive:** swerve widget (module states), speed graphs (target vs actual), heading.
- **Vision:** per camera: connected, tag count, accepted and rejected poses.
- **Tuning:** SysId and wheel-radius buttons, gain tunables.

### 1.9 Tests

- JUnit tests for pure logic: joystick shaping, vision filter and standard deviations, heading
  hold decision, alliance flip.
- A headless sim smoke run (`./gradlew run` with the `build/jni/release` libraries) that starts the
  robot, enables teleop and auto for a few seconds and checks the log for exceptions.

### 1.10 Done when

- `./gradlew build` passes with tests.
- The sim runs teleop and the BLine test auto with no exception; vision adds measurements in sim.
- Elastic opens the layout with all widgets bound to live topics.
- The branch is pushed to `nick-test`.

### Open questions for the robot (not blocking)

1. Steer and drive gains: run SysId (buttons in the Tuning tab).
2. Camera roll (each stream must show a level horizon) and the PhotonVision name of camera 4.
3. CAN utilization on `CAN_S2` at 100 Hz odometry (for a raise to 200 Hz).
4. Pro license on every device (issue #8).

---

## Sub-project 2: intake

### Hardware (CAD, registry and user, 2026-10-22)

- Rack: two Kraken X44, Talon 21 (turret side, left) and Talon 22 (other side, right), both on
  `CAN_S4`. Each drives its own side: 10T to 28T (2.8:1) to a toothed rack plate. No shaft links
  the sides. Stroke 324 mm along a 20 degree slope, out = downhill (CAD 2026-10-22: the plates
  move 304.3 mm forward and 110.8 mm down). Gravity pushes the rack out with about 9 N per side
  (old code's 5.4 kg rack): every rack request has a gravity term (`kG` Elevator_Static for
  Motion Magic, about 8 A added to the pushes and the rest), so the named currents are net.
  The old code's full stroke is 3.95 motor rotations; with the CAD stroke that is 12.19 motor
  rotations per meter (a check on the robot: push the rack out by hand, read the rotations).
- Rollers: Talon 20, Kraken X60, 24T to 36T (1.5:1). (The old sim used 0.5.)
- Talon 22 had a loose power connection at CAEL (up to 17 reboots a match). A reboot sets its
  position to 0 wherever the rack is.

### Rack control

- Units: meters of rack travel (0 = in, 0.324 = out). `SensorToMechanismRatio` = 12.19.
- Each side runs its own `MotionMagicVoltage` loop (FOC on) to the same goal. The old `Follower`
  copied the leader's output, not its position, so the sides drifted apart.
- Start gains (SysId or hand tuning replaces them): kS 0.25 V, kV 1.2 V per m/s (X44: 0.096 V
  per motor rps x 12.19), kP 30 V/m. Cruise 1.0 m/s, acceleration 6 m/s^2 (full stroke in
  about 0.5 s). Stator limit 40 A, supply limit 30 A per side.
- No soft limits: a side that was just resynced can be a centimeter or two off, and a soft limit
  would stop its push short of the hard stop. The Motion Magic goals stay 3 mm inside the stops,
  and the current limits protect the gears.

### Zero and stroke (user, 2026-10-22)

The robot is powered off in the pit and boots for the match with the intake in. So:

- **Zero at boot:** both sides read 0 = in. Nothing moves to find zero.
- **Out stop found on the first extend** (usually in auto): Motion Magic out to 0.289 m (CAD
  stroke 0.324 m less 3.5 cm), then each side pushes out gently with `TorqueCurrentFOC` at
  +6 A until it stops (speed under 0.005 m/s for 0.15 s). That position is the side's "out" for
  the rest of the match; later extends use Motion Magic to it, 3 mm short of the stop.
  Timeout 1.5 s for the push: alert "Intake rack did not reach the out stop", use the CAD
  stroke.
- **A rebooted side** (`hasResetOccurred()` on Talon 21 or 22; Talon 22 had a loose power
  connection) reads 0 wherever it is. Its position is set at once from the other side (the two
  sides move together, within the skew limit). It is marked "lost", raises an alert, and its next
  move confirms its position at that move's stop: on extend it pushes to the out stop and sets its
  position to the measured stroke; on retract it pushes to the in stop and sets 0. The other
  side moves normally.
- A measured out stop outside 85 % to 115 % of the CAD stroke means the rack was not in at
  boot: that side's position is set to the CAD stroke at the stop, with the alert "Intake was not
  in at boot".

### Hits, skipped teeth, rest (user, 2026-10-22)

The intake gets hit: it can be pushed in (the motors are backdriven) or skip rack teeth (the
position reading is then wrong). The code does not fight a hit and does not trust a position
after one:

- **Every move ends at a stop.** Motion Magic to 3.5 cm before the stop, then a push (6 A) until
  the rack stalls. If Motion Magic thinks it is there but the rack has not stalled, the push goes
  on: the rack is not at the stop yet.
- **Re-true at every stall:** in stop = 0; out stop = the measured stroke (the first out stall
  measures it). A stall more than 1 cm from where the side expected it means skipped teeth: info
  alert, position fixed.
- **Rest:** at the stop, a light 2 A push holds the rack there. A hit pushes it away with almost
  no power spent against it.
- **Pushed away:** more than 2 cm from the stop and still for 0.3 s: go back to the stop (Motion
  Magic first if more than 3.5 cm away, then the push) and re-true.
- **Blocked, not skipped** (review 2026-10-22): a stall more than 5 cm from the expected stop is an
  obstacle, not a skip: no re-true (it would be wrong by the whole distance), a MEDIUM alert, rest,
  and a new try after 2 s. A Motion Magic move that stops moving short of its target hands over to
  the push (it never holds 40 A against an obstacle). A side that is not confirmed at its stop
  tries again even if it drifted onto the stop, so a push confirms it. Booting with the rack out
  by more than 5 cm is reported, not corrected: check the rack is in before each match.
- **Push current 12 A** (6 A was about the rack's friction from kS, so a push could stall before
  the stop). Both Talons rebooting at once: each is set to its goal's stop and a push confirms it.
  Reboot flags in the first 3 s after code start are ignored (the devices' own boot).
- **All of this is per side** (user): one side can skip teeth while the other does not. Each side
  has its own push, stall check, re-true, rest and return.

### Skew guard (alert only)

- Skew = left position - right position. If |skew| > 2.5 cm for 0.1 s during Motion Magic:
  alert "Intake rack sides disagree" (skipped teeth or binding). The rack does not stop: a skip
  makes the readings differ while the rack is straight, and each side re-trues at its own stop at
  the end of the move, which clears the difference. (The rack carriage joins the two sides.)

### Rollers

- `MotionMagicVelocityVoltage` (FOC on), units roller rps (`SensorToMechanismRatio` 1.5). In
  26.7 roller rps, out -20 (the old 40 and -30 motor rps). kV 0.225 V per roller rps (the old
  0.15 per motor rps), kP 0.3. Supply limit 30 A. Brake when stopped.
- Jam: roller under 20 % of its target with stator current over 50 A for 0.25 s: run out for
  0.3 s, then in again; alert after 3 jams in 5 s.

### Commands and controls

- Left trigger (hold): home if needed, extend, run the rollers in. Release: rollers stop, rack
  stays out. A: rollers stop, rack retracts.
- Telemetry `Intake/...`: goal, left and right position, skew, homed, homing state, roller
  speed and target, jam count. The 3D pose of the intake follows the rack position.

### Sim

- Each rack side: `ElevatorSim` with no gravity and hard stops at 0 and 0.324 m, so the push
  into the out stop runs in sim. The sim's out stop is 6 mm past the CAD stroke on one side, to
  show that each side measures its own.
- Rollers: `FlywheelSim` with the 1.5 ratio.
- `tools/sim-smoke.sh` adds: the first extend measures both out stops, a later extend reaches
  them within 1 s, retract reaches 0.

### Tests

- `StallDetector` (pure): stalls only after the window; resets on movement.
- `SkewGuard` (pure): trips only after the time limit; clears on reset.
- Unit conversion: 3.95 motor rotations = 0.324 m.

---

## Sub-project 3: shooter suite

### Hardware (CAD gear trains, registry, user 2026-10-22)

| Part | Talon | Motor | Ratio (motor rot per output rot) | Units in code |
|---|---|---|---|---|
| Turret | 44 | X60 | 67.86 | turret rotations (0 = jig) |
| Hood | 41 | X44 | ~85 (5:1 planetary, 9T pinion, sector estimated from CAD) | hood degrees above the lower stop (Talon: hood rotations) |
| Flywheel | 42 + 43 (follower, opposed) | X60 x2 | 0.55 (the wheel turns faster) | wheel rps |
| Vertical roller ("kicker") | 40 | X60 | 1 | motor rps |
| Scotty rollers | 45 | X60 | 1.15 | motor rps |
| Indexer roller | 30 | X60 | 2.42 | motor rps |
| Belt tunnels | 31, 32 | X44 | not known | motor rps |

All on `CAN_S3`, except the indexer on `CAN_S1`. (The old turret code used `CAN_S1`.)

Values to measure on the robot: the hood ratio (hood from its lower to its upper stop by hand,
motor rotations in Tuner X), the hood's upper limit in degrees, the turret's cable limits (old
code: +-250 degrees from the jig), shot speeds and hood angles per distance, time of flight.

### Turret

- **Zero:** the turret is in its jig at power-on (user); the jig points the shooter to the
  robot's left (+y). So turret 0 = robot-relative +90 degrees. A Pit button "Zero turret (in jig)"
  sets 0 again.
- **Reboot:** a Talon 44 reboot loses the zero (relative encoder, no limit switch): alert HIGH,
  the turret goes neutral, aiming stops until the Pit button is pressed with the turret in the jig.
- **Control:** `MotionMagicExpoTorqueCurrentFOC` (Pro), gains in amps: kS 2 A, kP 360 A per
  turret rotation (1 A per degree), kD 15. Expo profile kV 8.1 V per turret rps (0.12 x 67.86),
  kA 0.6. Soft limits +-250 degrees (Talon), stator limit 40 A.
- **Aim math (pure, tested):** turret pivot in field = robot pose + `kTurretOffset` turned by the
  heading; field angle to the target; minus heading; minus 90 degrees (jig); then of the
  equivalent angles (+-360) inside the limits, the nearest to the present position. A lead of
  0.1 s of robot turn rate cancels the lag while the robot turns.
- Always tracks the target while enabled and zeroed (default command).

### Target and shot (ShotCalculator, pure, tested)

- **Target:** our hub while the robot is in our alliance zone; otherwise a pass target, depot or
  outpost by the robot's y (the old code's rule).
- **Distance** from the turret pivot to the target gives the flywheel speed and the hood angle
  (interpolated tables, separate for score and pass). The old tables are kept as start values
  (speeds converted from motor to wheel rps, x 1/0.55).
- **Shoot on the move:** the target is moved by -(robot field velocity x time of flight),
  iterated 3 times. Time of flight is a table too; until it is measured, 0 (shoot on the move
  off).

### Hood

- **Zero:** pushes down to its lower stop (TorqueCurrentFOC -3 A) at enable and after a reboot,
  like the intake rack; position set to 0 at the stall.
- **Control:** `MotionMagicVoltage` with `Arm_Cosine` kG (start 0: the hood angle above
  horizontal at the stop is not known; set `GravityArmPositionOffset` when measured). The Talon
  works in hood rotations; the code converts degrees. kP 180 V per hood rotation (0.5 V per
  degree), kS 0.2 V. Voltage, not torque current: a light
  mechanism with no measurements yet is easier to set up in volts.
- **Stow:** the hood stays down (0) unless the driver holds the shoot trigger, so the robot fits
  under the trench.

### Flywheel

- `VelocityTorqueCurrentFOC` (Pro) on Talon 42, Talon 43 follows opposed (one shaft). Wheel rps.
  kS 3 A, kP 5 A per wheel rps. Ready = within 3 % of the target for 0.1 s. Coast when off.

### Feed path

- Indexer roller (30), belt tunnels (31, 32), Scotty (45), vertical roller (40): all
  `MotionMagicVelocityVoltage`, speeds from the old code (motor rps). They run together only
  while the shot is ready (flywheel, hood and turret on target).

### Superstructure and controls (driver only)

- Turret: always tracks. Hood: stowed unless shooting.
- Right trigger (hold): flywheel to the table speed, hood to the table angle; feed when ready.
  Release: feed stops, flywheel coasts, hood stows.
- Y (hold): eject: feed path and intake rollers run out.
- Telemetry `Shooter/...`: target, distance, turret goal/position/error, hood goal/position,
  flywheel goal/speed, ready flags, feed running. Component poses: turret yaw and hood angle.

### Sim

- Turret: `DCMotorSim`; hood: `SingleJointedArmSim` with stops; flywheel and rollers:
  `FlywheelSim` (a shared `lib/sim/RollerSim`).
- Smoke test: in our zone, hold the trigger: the flywheel reaches its speed, turret error under
  1.5 degrees, hood on target, the feed starts.

### Changes after review (2026-10-22)

- No `setPosition(0)` in code for the turret or the intake rack: a Talon reads 0 at power-on
  (in the jig / intake in). A code restart keeps the Talon's true position; zeroing in code
  would have moved the zero (and the turret's cable limits) to wherever the mechanism was.
- "Ready" checks the turret against the true aim angle; only the command uses the 0.1 s turn lead.
- The feed starts on "ready" (turret steady for 0.1 s within 1.5 degrees, hood within 1, flywheel
  within 3 %) and keeps going while the shot is still good (flywheel above 85 %, turret within
  3 degrees, hood within 2), so it does not stop in the middle of every ball.
- Belt tunnels 90 motor rps (the old duty-cycle settings ran them flat out). Hood forward soft
  limit, and an alert when the hood strains short of its goal (a sign of a wrong kHoodRatio).
  On-target checks also need the Talon on CAN.
- Turret profile planned at half the motor's speed (Expo kV 16, kA 3). The turret sim runs every
  5 ms: with 20 ms steps the Talon's 1 kHz loop saw stale positions and the sim turret swung.

### CAD update (2026-10-22, export 89082afd)

- Swerve module positions = the steer axes in the CAD: (+-10.25, +-11.75) in (were 10 and 11.5).
- Turret axis (-6.0, +7.5) in. Intake slide 20 degrees (see sub-project 2).
- The AdvantageScope model was rebuilt from this export (it was built from the 2026-10-08 export:
  old belt motors and camera mounts). The model shows the intake OUT; `Final Poses` slides it back
  along the slope.

---

## Sub-project 4: autos

### Decisions (user, 2026-10-22)

- First routine: **Right double sweep** (the students' RightDoubleSwipe, blue frame; BLine turns it
  180 degrees for red).
- Start on a real start pose: (3.51, 2.66) m, heading 0 (intake toward the neutral zone), the
  students' start point. Every routine resets the pose to its first waypoint.
- No ball sensor: a `shoot` event feeds for a set time (`Auto/Shoot Time s`, start 2.5 s).

### Routine: "Right trench + bump x2" (user, 2026-10-22)

The user changed the route while it was built: under the trench with the intake down, intake,
back over the bump and shoot; out over the bump, a tighter sweep, back over the bump, shoot the
last hopper until auto ends. (It replaces the students' four-path RightDoubleSwipe, which did not
fit in 20 s.)

| Step | Path / command | Notes |
|---|---|---|
| 1 | `right-trench-1` | start (3.51, 2.66) heading 0; `intake` at once (out = lower, to fit under the trench); curve down to the trench line y 0.70, through the trench (3.0 m/s), sweep the center line (x 7.70), back over the right bump (2.0 m/s) to (3.40, 2.55); `prepare` on the bump |
| 2 | stop and shoot | for `Auto/Shoot Time s` (2.5) |
| 3 | `right-trench-2` | out over the bump, tighter sweep (x 7.20-7.65, y 1.95-3.20), back over the bump; `prepare` |
| 4 | shoot | until auto ends |

- Headings (user: the intake must face the way the robot goes when it collects): 0 through the
  trench; 90 for the sweep up the center line; back to 0 on the way home, before the bump, so
  neither path turns on the bump and path 2 crosses it outbound facing forward; the tight sweep
  leads its heading by one corner (30 then 90), because the heading controller lags a quick turn
  by ~50 degrees. Sim: intake 18 degrees off the travel direction on average, 38 at worst
  (was 90 and 179 with heading 0 everywhere).
- `prepare` = flywheel to speed and hood up, no feed, so the shot is ready when the robot stops.
- The robot stops to shoot: shoot-on-the-move is off until time of flight is measured.
- Segments that end in a sharp corner are slow (2.0-2.5 m/s): at 4 m/s BLine ran 1.1 m past a
  corner (over the center line) and 0.5 m past the trench line (into the wall) in the sim.
- Found in the sim: BLine rotation P 1.0 turned 180 degrees in ~5 s (now 3.0); drive kS 2 A could
  not beat friction on a slow turn (now 8 A), so a path never finished.

- Sim check (smoke test): start pose, intake out at the trench entry, lowest y and highest x
  (wall and center line), shots and feed time.

## Sub-project 5: fuel physics and the bump in the sim (user, 2026-10-22)

The user wants to see, in AdvantageScope, the balls picked up, pushed around, shot out of the
turret and scored in the hub, and the robot driving over the bump instead of through it.

maple-sim (the usual FRC game-piece sim) is not used: its newest release (v0.4.0-beta) targets
WPILib 2026.2.1, and WPILib 2027 renamed the packages. It would also replace the CTRE swerve sim
that the autos are tuned on. So the sim has its own small fuel model. Simulator only: nothing in
it runs on the robot.

### Geometry (measured from AdvantageScope's 2026 field model, so the sim matches the picture)

- Bumps: four, x 4.00-5.25 m (blue) and 11.29-12.54 m (red), y 1.283-3.440 and 4.629-6.786 m.
  A triangle in x: 0.165 m (6.5 in) high at the center (x 4.6255 / 11.9155), 15 degree ramps.
- Hub: solid up to its top, modeled as a 0.6 m radius cylinder around (4.625, 4.035) and the red
  mirror (the funnel top is a hexagon about 1.2 m across; a square box caught diagonal shots on
  its corners); the top is at 1.83 m (72 in); a ball that falls through a 0.53 m radius circle there scores.
- Fuel: 0.15 m, 0.23 kg. The field model stages 456 balls; the 408 on the field are the start
  positions (360 neutral zone, 24 in each depot; the 48 in the outpost chutes are left out).
  The robot starts with 8 (the preload).
- Robot (AdvantageScope robot model): bumpers about 0.82 x 0.90 m; intake front at x 0.325 m
  when in, plus the rack travel times cos 20 degrees when out; intake width 0.72 m.

### Behavior

- Floor balls roll with friction (1.0 m/s^2), roll down bump ramps, bounce off the walls, the
  hub base and each other. The robot's bumpers (and the intake, when it is out) push them.
- A ball that touches the intake, with the rack more than half out and the rollers running in,
  goes into the hopper (capacity 40, an assumption).
- While the feeder feeds, balls leave the hopper at 8 per second (an assumption), from the hood
  axis, along the turret direction, with the robot's velocity added. In flight: gravity and air
  drag (Cd 0.47). A ball that falls through the hub opening scores; it comes back out on the
  floor 0.9 m from the hub center, on the neutral-zone side (an assumption). A miss lands and
  rolls.
- Exit speed and launch angle: the user's default of 0.7 x the wheel surface speed cannot score
  with the old shot tables at any hood angle (the balls go almost straight up). The sim uses a fit
  to those tables instead: speed = 0.24 x the surface speed of the 4 in flywheel (CAD), launch
  angle = 78 degrees - hood angle. Both are fits, not measurements: replace them with a measured
  exit speed when there is one.
- Bump: the robot's height, pitch and roll come from the bump height under its four wheels. On a
  ramp the simulated robot covers cos(slope) x 0.9 of its wheel travel (the 0.9 is a slip
  assumption), so odometry runs ahead of the truth there and vision must correct it, as on the
  real robot. The CTRE sim has no chassis dynamics: the robot does not bounce or get stuck.
- AdvantageScope: `RobotPose3d` (the true robot pose with the bump) for the robot, and
  `Sim/Fuel` (Translation3d[]) for the balls. The vision sim uses the 3D pose too.
- The fuel resets (staged balls, preload of 8) at the start of auto and with "Sim/Reset Fuel".
- Smoke test: balls picked up, shots and balls scored in the teleop shot and in auto; highest
  robot z and pitch in auto.

### Collisions and the AdvantageScope layout (user, 2026-10-22)

The user saw no balls and a robot that clipped through the trench and the bump. Cause: the saved
AdvantageScope 3D tab drew the robot from `RobotPose` (2D, at floor height) and had no game-piece
source, so the field's staged balls stayed. `tools/advantagescope-sim-layout.json` (File > Import
Layout) sets it up: robot `RobotPose3d`, components `Final Poses`, odometry `RobotPose` as a green
ghost, `Sim/Fuel` as Fuel.

- `sim/field/FieldObstacles`: the true robot (bumpers, and the intake when out) stops at the hubs
  (x 4.029-5.223, y 3.438-4.631), the trench walls between each trench lane and its bump (y
  1.279-1.583 and 6.486-6.790), the tower uprights and the field walls; and at a trench arm
  (underside 0.565 m) when the robot is too tall: 0.559 m with the intake out, 0.67 m with it in
  (`sim/field/RobotOutline`, from the robot model).
- Found with it: path 1's lineup cut the corner of the trench wall. The robot now goes down to
  y 0.72 at x 3.15-3.2 (intake front still short of the wall) at 1.5 m/s, then along the lane.
- Under the trench arm the robot has 6 mm of headroom with the intake fully out (model heights).
  Check on the real robot and field.
- Fuel: below its panel tops (1.27 m) the hub is a 1.19 m square; scored balls leave through the
  gap above the rear panel (0.85 m) on the neutral-zone side.

### Trench and shooting rules (user, 2026-10-22)

User: under any of the four trenches the hood ("turret retracted to its lowest point") must be
down and the intake fully out, or the robot hits the arm; and the robot cannot shoot with the
intake in. Both in teleop and auto.

- `superstructure/TrenchGuard` (tested): near = the robot's center in a trench lane's y range and
  the robot within reach (0.65 m) + 0.25 m of the arm in x, now or within 0.6 s at its velocity.
  The wall between lane and bump means a robot on the bump is never "near".
- While near: `IntakeRack.holdOutWhile("trench")` extends the rack and ignores retract; the hood
  goal is 0 in shoot and prepare; no feed; HIGH alert if the intake is not fully out or the hood is
  above 1 degree.
- Shooting: `shoot()` holds the rack out ("shooting") and feeds only when `isExtended()` (each side
  within 1.5 cm of its out stop).
- Sim: the robot's height includes the hood (model: 0.528 m at 0 degrees, 0.636 m at 40; it
  reaches the arm at about 12 degrees). The AdvantageScope hood now tilts up as it rises (the old
  sign tilted it down).
- Smoke test: the teleop shot starts with the intake in (first feed with the rack fully out); a
  teleop run drives through the right trench with the intake in and the hood at 22 degrees: the
  intake is fully out and the hood at 0 under the arm, no contacts, no feed.

### Auto start and center lines (user, 2026-10-22)

User: a robot must start with some part over its starting line, so start in the trench lane on
the line with the intake facing the alliance wall; never fully cross the center line (touching is
fine); stay off the long center line, where our partner's auto runs.

- Field tape (AdvantageScope field model; its frame is WPILib's turned 180 degrees, which leaves
  every symmetric element where it was): blue starting line x 3.978-4.029 (the near faces of the
  hub, bumps and trench, out to the walls); center line x 8.245-8.296; long center line
  y 3.984-4.085.
- Start (3.80, 0.65) heading 180: the robot spans x 3.355-4.206 (over the line), 0.23 m short of
  the trench arm (4.435). The routine sets the pose, waits for the intake (`extend()`, 1 s at
  most), then follows path 1: backwards under the arm, intake trailing; it turns at x 6.10, once a
  turning corner (0.63 m) clears the trench, and sweeps x 7.55, y 1.30-3.05, intake leading.
- Path 2's tight sweep moved to x 7.05-7.40, y up to 3.00, at 1.5 m/s: the intake's front corner
  reaches 0.73 m from the robot center, and while the robot turned it reached x 8.34 and y 4.06.
- Sim (blue and red): farthest robot part x 8.21-8.22 (does not touch the center line), highest
  y 3.72-3.73, no field contacts, 35 balls picked up, 43 of 43 scored. The intake trails for about
  0.5 s after the trench exit while the robot turns (mean 46 degrees off the travel direction while
  collecting, from x 6.4).
- `Path.flip()` is deprecated in BLine beta.3: `setFlipped(red)`.

### Aim on the way home (user, 2026-10-22)

User: in the auto the turret waited until the robot was in place to turn to the target; it must
be aimed while the robot drives back into our zone, so it can shoot as soon as possible.

- Cause: outside our zone the shot target is a pass target (the outpost or depot), so on the way
  home the turret pointed there and swung about 120 degrees once the robot crossed into the zone.
- `ShotCalculator.calculate(..., zoneLookahead)`: the hub is the target if the robot is in our
  zone now or will be within the look ahead at its velocity (tested). The superstructure uses 1.0 s
  until the driver shoots, 0 while shooting (so a pass from the neutral zone still passes), and
  feeds a hub shot only from inside the zone.
- Sim (blue/red): turret 0-3 degrees off the hub when the robot enters the zone (was 119-125);
  first ball 0.04-0.24 s after the robot stops (was 0.24-0.26).
- Also: the smoke test's opening teleop steps now stand still in the blue zone (they drove over
  the bump, into the fuel and across the center line before the auto).

### Earlier turn after the trench (user, 2026-10-22)

User: after the trench exit the intake trailed for ~0.5 s while the robot turned; finish the turn
sooner so the intake leads into the neutral zone, keeping both center line margins.

- The turn now starts at (5.70, 0.70): with the intake out a turning corner reaches 0.73 m, and
  the intake swings toward the trench wall's corner (5.222, 1.279), 0.75 m away there; the back
  corners stay 7 cm off the field wall. The robot faces 90 degrees at (6.70, 1.10), then the sweep
  starts at (7.55, 1.40).
- BLine rotation P 3.0 -> 5.0: 3.0 trailed a fast turn by ~55 degrees (BLine's own rotation limits,
  720 deg/s and 1500 deg/s^2, were not the limit). Check for wobble on the robot.
- Sim (blue/red): heading within 15 degrees of 90 at x 6.96-6.98, 0.45 m before the first fuel
  (x 7.43); was x 7.62, inside the fuel. Up the sweep at most 3 degrees off 90 (was 14). Farthest
  part x 8.04-8.07, highest y 3.71-3.72, no contacts. Balls picked up unchanged (~35): the stretch
  where the intake trailed had no fuel in it.

### Left trench auto (user, 2026-10-22)

"Left trench + bump x2": the right route mirrored about the long center line for the high-y
trench, with the same start-on-the-line rule, trench guard and center line margins.

- `tools/mirror-paths.py` writes `left-trench-1/2.json` from `right-trench-1/2.json`
  (y -> 8.043 - y, heading -> -heading; 8.043 = the AndyMark field, as for BLine's red flip).
  Edit the right paths and run it; do not edit the left ones by hand.
- `AutoRoutines.trenchAndBump(side)` builds both routines: pose on the line, wait for the intake,
  path 1, shoot, path 2, shoot until auto ends.
- Smoke test: `SIM_SMOKE_AUTO=left` runs it and mirrors the robot onto the right side, so the
  same checks apply. Sim (blue/red): start x 3.355-4.206 over the line (field y 7.39), intake 100 %
  out before the arm, turn done at x 6.96, farthest part x 8.05-8.06, nearest the long center
  line 3.70-3.72 m from the right wall's frame (i.e. y 4.32-4.34, tape to 4.085), no contacts,
  44-45 of 44-45 scored.
- The depot and outpost are not mirrored on the field, but the route does not go near either.

### Auto review fixes (user, 2026-10-22)

From the whole-branch review: two auto defects that the sim did not show.

- **Intake not out at the start.** `extend()` ends when both rack sides rest, and a side also
  rests after a blocked or timed-out push. The routine now waits up to 2.0 s for
  `IntakeRack.isExtended()`. If the intake is still not fully out, the robot does not go under
  the trench: it follows `<side>-trench-fallback` (0.65 m back along the lane to x 3.15, out of
  the trench guard's range), raises the HIGH alert "Auto: the intake was not fully out...", and
  shoots until auto ends. The shot keeps the intake going out and feeds once it is out.
- **Requirements.** `Commands.defer(routine, Set.of(drive))` holds only the drive; the commands
  inside it held nothing, so the hood's default `stow()` could hold the hood at 0 during a shot
  when no `prepare` event held it. Each rack and shoot command in a routine now runs as a proxy
  (`asProxy()`), so the scheduler gives it its subsystems while it runs. The routine itself does
  not hold them: the path events (intake, prepare) use the same subsystems, and an event that
  takes one from the routine would cancel the whole routine.
- Smoke test: a match starts with the intake in (new step 15 retracts it before auto), so the auto
  now really waits for it. `SIM_SMOKE_RACK=blocked` blocks the left side at 0.10 m for the whole
  auto. Sim: normal run 43 of 43 scored, intake 100 % out at the trench, no contacts; blocked run:
  fallback taken, never under an arm, alert raised, no shot (the intake never got out); run with
  the `prepare` event removed: 44 of 44 scored.

### Rack trust and a dead rack Talon (user, 2026-10-22)

From the whole-branch review. The intake frame is not rigid (user): one side can sit short while
the other is out, so the other side tells nothing about this one.

- **Short or past (replaces "Hits, skipped teeth, rest" for the 1-5 cm band).** The motor's
  sensor cannot tell skipped teeth from something in the way, so `RackSide.stalledAt` decides by
  the direction of the error. Within 1 cm of the stop: set to the stop. 1-5 cm PAST the stop: a
  hard stop cannot be passed, so the reading is wrong: set to the stop, LOW "skipped teeth" alert.
  More than 5 cm past: not trusted. More than 1 cm SHORT: never fixed (something may hold the side
  there, and then the reading is right): the side does not count as out, MEDIUM alert, retry after
  2 s. A hit that pushes a side in and skips leaves the reading high, so it is fixed at the next
  out stop. A skip that leaves the reading low is fixed at the in stop.
- **Fixed out stops.** The first extend no longer measures the stroke (a ball on that extend
  would make a short stop the "out stop"). `IntakeConstants.kOutStopLeft/Right` are measured in
  the pit from `Intake/Rack/<side>/Last Stall m` (the reading at the stall, before any fix); CAD
  0.324 m until then. The 94-106 % plausibility band is gone.
- **Rebooted side.** No special trust: its copied reading follows the same rule (no skip alert
  when it is fixed).
- **Dead rack Talon** (user: "if a Talon is dead, we can't score... shoot what balls we already
  have"). A rack Talon not answering for 0.5 s (after the 3 s startup grace) is dead: its side's
  place is unknown, so `isExtended()` stays false (trench guard, trench alert and the auto's
  trench choice unchanged), and a HIGH alert says so. `Superstructure.allowed()` then does not
  wait for the intake (`isExtended() || hasDeadTalon()`); the shot still sends the working side
  out. No toggle.
- **Remaining limit.** A side pushed in by a hit whose gear skips, then held there, still reads
  its out stop: no motor sensor can see it. A switch at each out stop (for example on a CTRE
  CANdi, as a remote limit) would; a hardware choice, not in this change.
- Smoke test `SIM_SMOKE_RACK=short|dead|blocked`. Sim: normal 44 of 44 scored, intake 100 % out at
  the trench, no contacts, the teleop skip (3 cm) fixed as "past". short (right side 2.4 cm short)
  and blocked: fallback, never under an arm, no shot. dead (Talon 22 before auto): fallback, HIGH
  alert, 8 of 8 preload scored.

### Shooter and feed path review fixes (user, 2026-10-22)

From the review of the intake roller, feed path, turret, hood and flywheel.

- **Feed stop and jams** (`FeedLogic`, pure and tested; `Feeder` applies it). When the feed
  stops, stages 30, 31/32 and 45 brake at once and the vertical roller (40, into the wheel) runs on
  0.25 s, so it does not hold a ball against the spinning wheel. While feeding, a stage slower than
  20 % of its speed with more than 75 % of its stator limit for 0.25 s is a jam: every stage runs
  back (half speed) for 0.3 s, then feeds again; 3 jams in 5 s raise a MEDIUM alert naming the
  stage. Eject: all back, no jam check, no run-on. `isFeeding()` is false while unjamming.
- **Stator limits** on every feed stage: 60 A (X60), 40 A (X44), start values to measure.
- **Feed speeds** = the duty cycles the CAEL robot ran (7ba98388 `testRun`) as speeds, duty x 12 V
  / kV: indexer 96, top belt 125, bottom belt 112.5, Scotty 100.4, vertical roller 91.5 motor rps
  (were 25/90/90/30/30). Acceleration 600 rps/s. Per-stage speed and current are logged.
- **Flywheel**: `getSpeed()` is 0 and `isReady()` false while Talon 42 does not answer (a silent
  Talon keeps its last reading; CAEL Q40).
- **Turret**: soft limits 10 degrees inside the mechanical limit of +-270 (user, 2026-10-23; was +-360 wiring): +-260, aim +-255. After a Talon 44
  reboot the last good reading (connected, no jump over 15 degrees a loop) is written back
  (retried until the Talon takes it), the turret is neutral and not "on target" until the reading
  shows it, and a MEDIUM alert asks for a jig zero after the match. With no good reading the old
  behavior stays (turret stopped, HIGH alert). The jig zero retries `setPosition(0)` until it is
  taken.
- **Hood**: homing drives down at -1 V (`VoltageOut`; about 40 deg/s at 85:1) in place of a -3 A
  push that sped up to ~500 deg/s into the stop; stalled, 1 V still gives ~20 A, so a stall is the
  stop. The zero is set only when `setPosition(0)` is taken.
- Not changed (needs the robot first): hood kV and ratio, turret cold-boot check, flywheel idle
  spin, turret lead time, pass tables.
- Sim: 107 tests pass. Smoke normal 43 of 43 in auto, teleop shot 6 of 8, no feeder jams, hood
  homes; red left 44 of 44; dead rack Talon 8 of 8 preload; short side no shot. The sim's feed
  rollers have no ball load, so a jam never happens in the sim: the jam logic is covered by
  `FeedLogicTest` only, and the turret restore by reading only (the sim cannot reboot a Talon).

### Shoot on the move (user, 2026-10-22)

Tuned in the sim (the robot must measure the time of flight, the latency and the lead again).

- **Tables from the sim's ball model** (`sim/field/ShotModel`, checked by `ShotTablesTest`): the
  flywheel speed (wheel rps) at which a still shot lands on the hub center at each distance
  1.25-6.0 m, with the old hood table (1.5 m: 6 degrees; 10 hit the rim); the time of flight from
  the same model. The old speeds are kept in a comment.
- **`ShotCalculator`**: the ball takes the turret axis velocity (robot + omega x offset); the shot
  is aimed from where the robot will be when the ball leaves (`kShotLatency` 0.02 s); aim point =
  target - v x tof x (1 - 0.10 tof) (drag slows the drift; 0 left balls behind, 0.19 ahead),
  solved until it settles. Turret goal = aim + (bearing rate - omega) x 0.05 s; hood and flywheel
  goals for the distance 0.1 s / 0.08 s ahead; "ready" checks the values needed now.
- **Gate**: turret within 0.15 m sideways at the hub (min 1 degree), robot under 3.2 m/s and
  2.2 rad/s, aim distance inside the tables (1.25-6.0 m). Dashboard "Shooter/Shoot On The Move"
  off: no lead, shots only when still. "Shooter/Blocked By" names the check that stops a shot.
- **Sim test** `tools/sim-sotm.sh` (still, strafe, toward, away, arc at 1-4 m/s, spin): every ball
  shot scored up to 3 m/s; mean miss 0.06-0.10 m strafing, 0.23 m on a 3 m/s arc, 0.36 m strafing
  at 2 m/s while spinning at 2 rad/s. With the lead off: 0 % at 1 m/s.

### Double trench auto, shooting on the move (user, 2026-10-22)

"Right/Left trench x2, shoot on the move" (paths `right-trench-sotm-1/2`, left mirrored).

- Start on the line 0.18 m farther from the trench (x 3.62, the robot spans the line tape) and go
  at once: the intake goes out while the robot drives. The **trench brake** (`TrenchGuard.mustBrake`,
  in the auto's drive output) stops the robot if it could not stop (4 m/s^2, 0.1 m short) before a
  trench arm while the intake is not out or the hood not down. So the trench passes run at 3.5 m/s
  and the robot is under the arm at t 0.5 s (was 1.0 s). If the intake is still not out after 2 s
  while the brake holds, the trench is given up (`right-trench-sotm-fallback`) and it shoots.
- Sweep the x 7.55 column, back over the bump fast (3 m/s from its top to past its edge: slow, the
  robot can get stuck on top), slow hard, then straight on and straight to the trench entrance at
  0.8 m/s, shooting on the move (the "shoot" event waits for our zone, ends near the trench).
  West of x 3.6 before y 2.0, so the bumper clears the trench wall. While shooting the robot turns
  180 -> 90 -> 0, so it goes through the trench intake first and reaches the fuel facing it (it
  cannot be diagonal in the 1.28 m lane). Through the trench at 3.5 m/s, turning into the x 7.40
  column without stopping, back home the same way, shooting; stop at the entrance and shoot the
  rest.
- Sim, both sides, both alliances: 41-42 of 41-42 scored, 2.5-3.2 s of feeding on the move, no
  contacts, farthest part x 8.04-8.06, highest y 3.67-3.73. The first sweep fills the hopper (40):
  the second finds almost no fuel left in our quadrant.
