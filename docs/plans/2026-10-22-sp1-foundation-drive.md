# Sub-project 1: foundation and drive, implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A logged, simulated, dashboard-ready robot base: closed-loop Pro swerve, working four-camera
vision, health checks and an Elastic layout, on `nick-test`.

**Architecture:** Real `TalonFX` objects in sim and on the robot (CTRE sim thread for the swerve).
Pure logic in small classes with JUnit tests. Telemetry through `org.wpilib.telemetry.Telemetry`
(NetworkTables `/Telemetry`), recorded to `.wpilog` by `DataLogManager`; CTRE signals to hoot.

**Tech Stack:** Java 25, WPILib 2027.0.0-alpha-7, Phoenix 6 26.70.0-alpha-2 (Pro, classic CAN),
PhotonLib 2027 alpha-2, BLine-Lib 2027.0.0-beta.3, Elastic 2027.0.0-alpha9.

**Spec:** `docs/specs/2026-10-22-nick-test-rewrite-design.md` (section "Sub-project 1").

## Global Constraints

- Build and test: `JAVA_HOME=~/wpilib/2027_alpha7/jdk sh ./gradlew build` (runs JUnit).
- No CAN FD features: no time sync, no Dynamic Motion Magic, odometry 100 Hz.
- 2027 names: `ChassisVelocities`, `SwerveModuleVelocity`, `ApplyRobotVelocity`,
  `setOperatorForwardDirection`, state fields `Velocity` (robot-relative), `ModuleVelocities`,
  `ModuleTargets`, `RawHeading`.
- Every swerve request sets `SteerRequestType.MotionMagicExpo` (the library default is `Position`).
- Vision timestamps go to `addVisionMeasurement` unchanged (same clock on SystemCore).
- Alerts: always `AlertPublisher.create(group, id, text, level)`; ids unique (a duplicate id
  throws at startup).
- Telemetry names: `<Subsystem>/<Name>`; Tunables: `<Subsystem>/<Name>`.
- Commit messages explain the problem, the concept and the fix (students read them). No AI
  attribution lines.
- Robot-only values are named constants with a comment that says how to measure them.

## Review Focus

1. **Camera disconnected mid-match:** `getAllUnreadResults()` returns empty; vision must add
   nothing and raise the camera alert, no exception. Test in Task 7 (empty result list).
2. **Alliance unknown (no DS):** heading targets, flips and the operator perspective must use
   blue and not throw on an empty `Optional`. Test in Task 5 (`HeadingHold` and hub angle with no
   alliance).
3. **Stick released while turning:** heading hold must capture the heading only after the
   robot's turn rate falls below the threshold, or the robot snaps back. Test in Task 5.
4. **Open-loop fallback switch flipped while driving:** the request type changes on the next
   loop with no jump in the steer. Covered by the sim run in Task 11.
5. **Pose reset during BLine start:** BLine reads the pose right after its reset; the supplier
   must be live (`getState().Pose`). Checked in the Task 11 sim auto run (the path starts at the
   reset pose, log `Auto/BLine/...`).

---

### Task 1: AlertPublisher

**Files:**
- Create: `src/main/java/frc/robot/lib/util/AlertPublisher.java`
- Test: `src/test/java/frc/robot/lib/util/AlertPublisherTest.java`

**Interfaces:**
- Produces: `static Alert create(String group, String id, String text, Alert.Level level)`;
  `static String[] active(String group, Alert.Level level)`; `static void publish()` (logs
  `Alerts/<group>/errors|warnings|infos` and sets the table type `Alerts`). HIGH = errors,
  MEDIUM = warnings, LOW = infos.

- [ ] Step 1: tests `createdAlertIsListedWhenSet` (set true, `active("G", HIGH)` contains the text;
  set false, empty) and `levelsGoToTheirOwnList`.
- [ ] Step 2: run `./gradlew test`: fails to compile.
- [ ] Step 3: implement; keep a `List<Alert>` per group in a `LinkedHashMap`.
- [ ] Step 4: tests pass.
- [ ] Step 5: replace every `new Alert(` in `utils/HealthMonitor.java` and `Vision.java` with
  `AlertPublisher.create(`; build passes. Commit.

### Task 2: Constants split and Phoenix helper

**Files:**
- Create: `constants/CanIds.java` (all CAN ids and `CANBus` objects from `Constants.CANIDConstants`,
  same names), `constants/FieldConstants.java` (from `Constants.FieldConstants`, plus
  `Field kField = Fields.FRC_2026_REBUILT_ANDY_MARK.loadField()` and
  `Translation2d hub(boolean red)`), `lib/phoenix/PhoenixUtil.java`.
- Modify: `Constants.java` (delete the moved classes), every user of them (imports only).

**Interfaces:**
- Produces: `PhoenixUtil.applyConfig(String name, Supplier<StatusCode> apply)` tries up to 5
  times and raises an alert `Phoenix/<name>/config` "Config failed: <name>" (HIGH) on failure;
  `FieldConstants.kField`, `FieldConstants.hub(boolean red)`.

- [ ] Step 1: move the classes; `./gradlew build` passes.
- [ ] Step 2: commit.

### Task 3: Robot loop, logging, web server

**Files:** Modify `Robot.java`.

- [ ] Step 1: in the constructor: `DataLogManager.start()`, `DriverStation.startDataLog(DataLogManager.getLog())`,
  `WebServer.start(5800, Filesystem.getDeployDirectory().getPath())`; on the robot only
  `SignalLogger.start()` (sim: `SignalLogger.enableAutoLogging(false)`).
- [ ] Step 2: `robotPeriodic`: keep `HootAutoReplay.update()`, PDH logging; add
  `Telemetry.log("Match Time", MatchState.getMatchTime())`, `AlertPublisher.publish()`, the
  component poses (`Final Poses`: turret, hood, intake at their zero poses until SP2/3).
  Remove the commented test code.
- [ ] Step 3: `teleopInit` cancels the auto command (review item from Issues3.md).
- [ ] Step 4: sim smoke run (Task 11 command) shows no exception; commit.

### Task 4: Drive subsystem and Pro gains

**Files:**
- Create: `subsystems/drive/Drive.java` (replaces `SwerveSubsystem.java`), `subsystems/drive/DriveConstants.java`.
- Modify: `generated/TunerConstants.java`, every user of `SwerveSubsystem`.

**Interfaces:**
- Produces on `Drive`: `Pose2d getPose()` (live `getState().Pose`), `ChassisVelocities getRobotVelocity()`,
  `ChassisVelocities getFieldVelocity()`, `Rotation2d getHeading()`, `void resetPose(Pose2d)`,
  `Command applyRequest(Supplier<SwerveRequest>)`, `void driveRobotRelative(ChassisVelocities)`
  (closed-loop `ApplyRobotVelocity`), `static boolean isRedAlliance()`, `SwerveDriveState getStateCopy()`,
  `Command sysId(Routine, SysIdRoutine.Direction, boolean dynamic)` with `enum Routine {TRANSLATION, STEER, ROTATION}`,
  `Command wheelRadiusCharacterization()`.
- `DriveConstants`: `kMaxSpeed` (= `kSpeedAt12Volts`), `kMaxAngularRate` 1.85 rot/s in rad/s,
  `kStickDeadband` 0.08, `kTranslationExponent` 2, `kRotationExponent` 3, `kHeadingP` 6.0,
  `kHeadingHoldRateThreshold` 0.15 rad/s, `kDriveBaseRadius` from the module positions.

- [ ] Step 1: `TunerConstants`: steer gains kP 20, kD 0.1, kS 0.1, kV 0.68; drive gains kP 3, kS 2,
  kV 0; `kDriveClosedLoopOutput = TorqueCurrentFOC`. Comments say "start values, replace with SysId".
- [ ] Step 2: write `Drive` from `SwerveSubsystem`: keep the sim thread, operator perspective,
  SysId routines, wheel-radius test; drop the `getState().Pose = ...` lines, `cachedPose`, the
  shoot-on-the-move code (moves to SP3) and the commented code. All requests set MotionMagicExpo.
- [ ] Step 3: build passes; commit.

### Task 5: Teleop drive commands and heading hold

**Files:**
- Create: `subsystems/drive/HeadingHold.java` (pure), `commands/DriveCommands.java`.
- Test: `src/test/java/frc/robot/subsystems/drive/HeadingHoldTest.java`.

**Interfaces:**
- `HeadingHold`: `Optional<Rotation2d> update(double rotationCommand, double measuredRate, Rotation2d heading, boolean translating)`
  returns the heading to hold, or empty when the driver turns; `void reset()`.
- `DriveCommands`: `static Command teleop(Drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, BooleanSupplier openLoop)`,
  `static Command snapTo(Drive, x, y, Rotation2d target)`, `static Command faceHub(Drive, x, y)`,
  `static Command xLock(Drive)`.

- [ ] Step 1: tests: `driverTurningGivesNoHold`; `holdCapturedAfterRateSettles` (rate 1.0 → empty,
  rate 0.05 → present, equal to the heading at that call); `holdIsKeptWhileTranslating`;
  `noHoldWhenStoppedAndNotTranslating`; `resetClearsHold`.
- [ ] Step 2: run, fail; implement; pass.
- [ ] Step 3: `teleop`: `JoystickShaping` on each axis (deadband `kStickDeadband`, exponents 2 and 3),
  scale by `kMaxSpeed` and `kMaxAngularRate`; drive with `FieldCentric` when turning, or
  `FieldCentricFacingAngle` (`withHeadingPID(kHeadingP, 0, 0)`) while `HeadingHold` returns a heading;
  `DriveRequestType.Velocity` unless `openLoop` is true. The tunable `Drive/Open loop teleop`
  (default false) gives `openLoop`.
- [ ] Step 4: `faceHub` target = angle from the robot to `FieldConstants.hub(isRedAlliance())`,
  blue when the alliance is unknown; test `hubAngleWithNoAlliance` in `HeadingHoldTest` (static
  helper `DriveCommands.angleToHub(Pose2d, boolean red)`).
- [ ] Step 5: build, tests pass; commit.

### Task 6: Drive telemetry

**Files:** Create `subsystems/drive/DriveTelemetry.java`; Modify `Drive.java` (register it).

**Interfaces:** `void update(SwerveDriveState state)` called from `Drive.periodic()`.

- [ ] Step 1: log `Drive/Pose` (Pose2d), `Drive/Module Velocities` and `Drive/Module Targets`
  (`SwerveModuleVelocity[]`), `Drive/Velocity` (ChassisVelocities), `Drive/Speed mps`,
  `Drive/Odometry Hz` (1/OdometryPeriod), `Drive/Failed DAQs`.
- [ ] Step 2: table `Swerve Drive` (type `SwerveDrive`): the nine doubles, angles in radians.
- [ ] Step 3: table `Field` (type `Field2d`): `Robot` single Pose2d; vision arrays come from Task 7.
- [ ] Step 4: sim run shows the topics (Task 11 check); commit.

### Task 7: Vision rewrite with four cameras

**Files:**
- Create: `subsystems/vision/Vision.java`, `subsystems/vision/VisionCamera.java`, `subsystems/vision/VisionConstants.java`.
- Delete: `subsystems/drive/Vision.java`; `Constants.VisionConstants`.
- Test: `src/test/java/frc/robot/subsystems/vision/VisionCameraTest.java`.

**Interfaces:**
- `VisionConstants.CameraConfig(String name, Transform3d robotToCamera)`; `kCameras` list with the
  four spec poses (inches and degrees converted with `Units`), names `front_right`, `front_left`,
  `side_left`, `side_right`.
- `VisionCamera.process(List<PhotonPipelineResult> results)` returns `List<Measurement>`, with
  `record Measurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs)`; it also keeps the
  last accepted and rejected poses for telemetry.
- `Vision(Drive)`: `periodic()` reads `camera.getAllUnreadResults()` once per camera, calls
  `process`, adds each measurement in time order, logs `Vision/<name>/...`, updates the sim.

- [ ] Step 1: tests (with `PhotonPipelineResult` built from a `PhotonCameraSim` or hand-made
  results): `emptyResultsGiveNoMeasurements`; `eachResultGivesItsOwnMeasurement` (two results,
  two measurements with their own timestamps); `rejectedPoseIsNotReturned`.
- [ ] Step 2: run, fail; implement (multi-tag, else lowest ambiguity; `VisionFilter.evaluate`);
  pass.
- [ ] Step 3: `Vision` logs per camera: `Connected`, `Tag Count`, `Accepted Pose`, `Rejected Pose`,
  `Reject Reason`, and the robot roll, pitch and z of each estimate (`Vision/<name>/Robot Roll deg`,
  `Robot Pitch deg`, `Robot Z m`: a wrong camera angle shows as a robot roll or pitch).
- [ ] Step 4: sim: `VisionSystemSim` with `FieldConstants.kField`, each `PhotonCameraSim` built with
  that field; truth pose from `DriveSim` (Task 8). Set `FlippingUtil.fieldSizeX/Y` from the field.
- [ ] Step 5: build, tests pass; commit.

### Task 8: Drive sim truth and pose-kick test

**Files:** Create `subsystems/drive/DriveSim.java`; Modify `Drive.java`.

**Interfaces:** `Pose2d truthPose()` (odometry-only pose from module positions and raw heading);
tunable command `Sim/Kick Pose` (adds 0.5 m and 10 degrees to the estimator pose).

- [ ] Step 1: implement with `SwerveDriveOdometry` on `getKinematics()`.
- [ ] Step 2: sim: press `Sim/Kick Pose`, vision pulls `Drive/Pose` back to `Sim/Truth Pose` within
  2 s (check in the Task 11 run); commit.

### Task 9: Health

**Files:** Move `utils/HealthMonitor.java` to `health/HealthMonitor.java`; port
`utils/SwerveHealth.java` from `origin/feat/swerve-health` (a2914a6) to `health/SwerveHealth.java`
with `Drive` in place of `SwerveSubsystem`; apply that branch's `HealthMonitor` changes.

- [ ] Step 1: port; publish `Pit/Swerve Self-Test` and `Pit/Wheels Forward` with `Tunables.publish`.
- [ ] Step 2: build; commit.

### Task 10: BLine autos and RobotContainer

**Files:** Create `autos/AutoRoutines.java` (replaces `utils/Autos.java` and `PathPlanningThing/SelectAuto.java`);
rewrite `RobotContainer.java`.

**Interfaces:** `AutoRoutines(Drive)`: `Selectable<Command> chooser()` published as `Auto Selection`;
BLine builder with new `PIDController`s (translation 2.0, rotation 1.0, cross-track 0.2), pose supplier
`drive::getPose`, measured velocity `drive::getRobotVelocity`, output `drive::driveRobotRelative`,
`withDefaultShouldFlip()`, `withTelemetry(Telemetry.getTable("Auto/BLine"))`. Each routine is built
fresh by a `Supplier<Command>` (a composed command cannot be reused).

- [ ] Step 1: `RobotContainer`: driver gamepad 0: sticks = teleop; A = face hub; B = X-lock;
  D-pad = snap 0/90/180/270; Start = reset heading (`seedFieldCentric`); left trigger and face-down
  keep the intake bindings. Test gamepad 2: SysId (A/B/X/Y = quasistatic and dynamic, forward and
  reverse; bumpers pick the routine shown as `Drive/SysId Routine`). Dashboard commands under
  `Tuning/...` for SysId and the wheel-radius test.
- [ ] Step 2: build; commit.

### Task 11: Elastic layout and sim verification

**Files:** Create `src/main/deploy/elastic-layout.json`; Create `tools/sim-smoke.sh`.

- [ ] Step 1: layout tabs Driver, Pit, Drive, Vision, Tuning per spec 1.8 (topics from Tasks 1-10).
- [ ] Step 2: `tools/sim-smoke.sh`: builds, runs `./gradlew run` headless with
  `DYLD_LIBRARY_PATH` and `JAVA_TOOL_OPTIONS=-Djava.library.path=build/jni/release`, sets
  `SIM_SMOKE=1`, and fails on `Exception` in the output. With `SIM_SMOKE=1`, `Robot.simulationPeriodic`
  uses `DriverStationSim` to run 10 s teleop with a scripted stick, 15 s auto, a pose kick, then
  `System.exit(0)`.
- [ ] Step 3: run it; also run the GUI sim, open Elastic with the layout, and check each widget has
  data; check the Review Focus items 4 and 5.
- [ ] Step 4: commit; push `nick-test`.
