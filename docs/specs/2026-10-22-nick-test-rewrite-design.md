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
  the sides. The rack is level (CAD z 382 mm at both ends): no gravity term. Stroke 324 mm (CAD).
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
