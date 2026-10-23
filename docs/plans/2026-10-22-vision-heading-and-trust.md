# Vision clock, trust, heading and single-tag pick: implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Vision gives the drive correct capture times, sensible trust, the robot heading from multi-tag frames, and the correct single-tag solution.

**Architecture:** All decisions stay in pure classes with unit tests (`VisionFilter`, new `VisionClock`). `VisionCamera` turns PhotonVision frames into measurements. `Vision` reads the clocks and the heading once per loop, converts times, and feeds the Phoenix pose estimator. `Drive` gets explicit odometry standard deviations. The sim smoke test fails when the kicked pose or heading is not corrected. Task order differs from the review: the σ model (review change 3) comes before the heading (change 2), because the heading σ uses the new 1/n² model.

**Tech Stack:** Java 21, WPILib 2027.0.0-alpha-7 (`org.wpilib.*`), Phoenix 6 26.70.0-alpha-2 (`SwerveDrivetrain`), PhotonLib dev-v2027.0.0-alpha-2-80-g82f7f4c3, JUnit 5.

**Spec:** `vision-review-vs-6328.md` (same folder as this plan; also in the shared notes folder), changes 1 to 4 and risks R1 to R4. Design history: `docs/specs/2026-10-22-nick-test-rewrite-design.md` section 1.7.

**Where this plan goes:** copy it to `docs/plans/2026-10-22-vision-heading-and-trust.md` in the first commit (the repo keeps its plans in `docs/plans/`).

## Global Constraints

- Start from the tip of `nick-test` after the `qol-calibration` session has committed. It edits `TunerConstants.java`, `RobotContainer.java`, `tools/sim-smoke.sh`, `tools/make-elastic-layout.py` and `src/main/deploy/elastic-layout.json` now. Work in a separate worktree; line numbers below are from 7c662d3 and can have moved: find code by name.
- Comments and docs: plain short sentences, the same style as the surrounding code (see `VisionFilter.java`).
- No Claude attribution in commits. Commit messages: Conventional Commits, as in `git log`.
- Units: meters, radians, seconds in code; degrees and ms only in telemetry names that say so.
- Vision heading on the real robot stays OFF until the camera check (review change 2): `VisionConstants.kCamerasChecked = false`. The sim always trusts it (its cameras are exact by construction).
- Values (start points, tuned later from real logs): odometry σ (0.003 m, 0.003 m, 0.002 rad); vision xy σ = max(0.02, 0.01·d²/n²); θ σ = max(0.02, 0.03·d²/n²); heading only for n ≥ 2 and d ≤ 4.0 m; max frame age 0.5 s; max future 0.01 s; single-tag max heading error 15°; ambiguity limit stays 0.2.
- Verify each task with `./gradlew test` (JDK: `JAVA_HOME=~/wpilib/2027_alpha7/jdk`). Tasks 2 and 3 also run `sh tools/sim-smoke.sh` (prints PASS or FAIL).

## Review Focus

- A Pigeon reboot makes the gyro heading wrong by tens of degrees: multi-tag frames must still be accepted and must correct the heading (Task 4 test `multiTagIgnoresTheGyroHeading`).
- Clocks with different epochs (Phoenix vs NetworkTables): the conversion must give the right drive time for any offset, also a negative one (Task 1 test `convertsWithANegativeOffset`).
- A coprocessor just rebooted, time sync not converged: frames with an absurd age must be rejected and not reach the estimator (Task 1 tests `rejectsOldFrame`, `rejectsFutureFrame`).
- A multi-tag frame whose Pi used fewer tags than it saw: n must come from `fiducialIDsUsed` (Task 2 test `multiTagCountsOnlyTheTagsInTheSolve`).
- The robot sees one tag only after the kick: the heading cannot come back, and the smoke test must say why it failed (Task 3 prints the count of heading measurements).

---

### Task 1: Clock conversion, frame age and latency telemetry

**Files:**
- Create: `src/main/java/frc/robot/subsystems/vision/VisionClock.java`
- Modify: `src/main/java/frc/robot/subsystems/vision/VisionCamera.java` (`process`, telemetry getters)
- Modify: `src/main/java/frc/robot/subsystems/vision/Vision.java` (`periodic`, `log`)
- Modify: `tools/make-elastic-layout.py` (Vision tab), then regenerate `src/main/deploy/elastic-layout.json`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionClockTest.java`, `VisionCameraTest.java`

**Interfaces:**
- Produces:
  - `final class VisionClock` with `static final double kMaxAge = 0.5`, `static final double kMaxFuture = 0.01`,
    `static double toDriveTime(double captureSeconds, double driveNowSeconds, double visionNowSeconds)` = capture + driveNow − visionNow,
    `static String ageRejectReason(double ageSeconds)`: `"old"` if age > kMaxAge, `"future"` if age < −kMaxFuture, else `""`.
  - `VisionCamera.process(List<PhotonPipelineResult> results, double visionNowSeconds)` (replaces `process(List)`). `Measurement.timestamp()` stays in the vision (NetworkTables) clock.
  - `double VisionCamera.lastLatencySeconds()`: age of the newest frame of the last call, NaN if none.

- [ ] **Step 1: Write the failing tests**

`VisionClockTest`:
```java
@Test void convertsWithAPositiveOffset() { assertEquals(15.0, VisionClock.toDriveTime(10.0, 105.0, 100.0), 1e-9); }
@Test void convertsWithANegativeOffset() { assertEquals(7.0, VisionClock.toDriveTime(10.0, 97.0, 100.0), 1e-9); }
@Test void acceptsNormalLatency() { assertEquals("", VisionClock.ageRejectReason(0.035)); }
@Test void rejectsOldFrame() { assertEquals("old", VisionClock.ageRejectReason(0.6)); }
@Test void rejectsFutureFrame() { assertEquals("future", VisionClock.ageRejectReason(-0.02)); }
@Test void allowsSmallClockJitter() { assertEquals("", VisionClock.ageRejectReason(-0.005)); }
```
`VisionCameraTest`: change every `process(list)` call to `process(list, 10.03)` (`eachResultGivesItsOwnMeasurement`: `10.06`), and add:
```java
@Test void oldFrameIsRejected() {
    assertTrue(m_camera.process(List.of(resultAt(10.0, 0.05)), 11.0).isEmpty());
    assertEquals("old", m_camera.lastRejectReason());
}
@Test void latencyIsTheAgeOfTheNewestFrame() {
    m_camera.process(List.of(resultAt(10.0, 0.05), resultAt(10.03, 0.05)), 10.05);
    assertEquals(0.02, m_camera.lastLatencySeconds(), 1e-6);
}
```

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `./gradlew test --tests 'frc.robot.subsystems.vision.*'`
Expected: FAIL, compile errors (`VisionClock` and `process(List, double)` do not exist).

- [ ] **Step 3: Implement `VisionClock` and the new `VisionCamera.process`**

`process` checks the age first (`visionNowSeconds - result.getTimestampSeconds()`); a frame with a reason is rejected with that reason and adds no pose to the rejected list (there is no estimate). Clear `m_lastRejectReason` at the start of each call (fixes the stale reason, R8).

- [ ] **Step 4: Wire the clocks in `Vision.periodic`**

Read once per loop, before the camera loop: `double visionNow = NetworkTablesJNI.now() * 1e-6;` (`org.wpilib.networktables.NetworkTablesJNI`; PhotonLib stamps frames in this clock) and `double driveNow = Utils.getCurrentTimeSeconds();` (`com.ctre.phoenix6.Utils`; the clock `addVisionMeasurement` needs). Pass `VisionClock.toDriveTime(m.timestamp(), driveNow, visionNow)` to `addVisionMeasurement`. Delete the comment "Same clock on the SystemCore". Log:
- `Vision/Clock Offset ms` = (driveNow − visionNow) × 1000
- `Vision/Timer Offset ms` = (driveNow − `Timer.getTimestamp()`) × 1000
- `Vision/<cam>/Latency ms` = `lastLatencySeconds()` × 1000 (skip the log when NaN)

- [ ] **Step 5: Add the three topics to the Vision tab**

In `tools/make-elastic-layout.py`: a `text("Latency ms", base + "Latency ms", ...)` per camera, and one text widget each for `/Telemetry/Vision/Clock Offset ms` and `/Telemetry/Vision/Timer Offset ms`. Run `python3 tools/make-elastic-layout.py src/main/deploy/elastic-layout.json`.

- [ ] **Step 6: Run the tests and the smoke test**

Run: `./gradlew test` then `sh tools/sim-smoke.sh`
Expected: tests PASS; smoke prints `PASS` (the layout check finds the new topics). In `build/sim-smoke.log` the end of step 7 still shows an error ≤ 0.05 m. Assumed: in the sim, PhotonCameraSim stamps frames with `wpi::Now`, the same clock as `NetworkTablesJNI.now()`. If the step 7 error grows, or the sim log shows "old" or "future" rejects, that assumption is wrong: stop and report.

- [ ] **Step 7: Commit**

```bash
git add docs/plans/2026-10-22-vision-heading-and-trust.md src/main/java/frc/robot/subsystems/vision src/test/java/frc/robot/subsystems/vision tools/make-elastic-layout.py src/main/deploy/elastic-layout.json
git commit -m "fix(vision): convert capture times to the Phoenix clock, reject stale frames, log latency"
```

- [ ] **Step 8 (on the robot, later): Record the offsets**

Robot on, both Pis up, 2 minutes. Read `Clock Offset ms`, `Timer Offset ms` and each `Latency ms`. Expected: the offsets are steady within 1 ms; latency 20 to 80 ms. Write the values into the CAEL-style notes. A drifting offset means a clock problem: stop and investigate before Task 3 goes on the robot.

---

### Task 2: Explicit odometry σ and the 1/n² vision σ model

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/DriveConstants.java`, `Drive.java` (constructor), `src/main/java/frc/robot/generated/TunerConstants.java` (`createDrivetrain` only)
- Modify: `src/main/java/frc/robot/subsystems/vision/VisionFilter.java`, `VisionCamera.java`, `VisionConstants.java`
- Test: `VisionFilterTest.java`, `VisionCameraTest.java`

**Interfaces:**
- Consumes: `VisionCamera.process(List, double)` from Task 1.
- Produces:
  - `DriveConstants.kOdometryStdDevs = VecBuilder.fill(0.003, 0.003, 0.002)`, `DriveConstants.kDefaultVisionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9)` (`Matrix<N3, N1>`).
  - `Drive(SwerveDrivetrainConstants c, double odometryHz, Matrix<N3,N1> odometryStdDevs, Matrix<N3,N1> visionStdDevs, SwerveModuleConstants<?,?,?>... modules)` replaces the old constructor. `createDrivetrain` passes `0` (Phoenix default: 100 Hz on CAN 2.0) and the two constants.
  - `VisionConstants.CameraConfig(String name, Transform3d robotToCamera, double stdDevFactor)`; all four cameras use 1.0.
  - `VisionFilter`: `kXyCoefficient = 0.01`, `kThetaCoefficient = 0.03`, `kMinXyStdDev = 0.02`, `kMinThetaStdDev = 0.02`; σ = max(min, coefficient·d²/n²).

- [ ] **Step 1: Write the failing tests**

`VisionFilterTest`:
- `moreTagsAreTrustedMore`: change the assertion to `assertEquals(two.xyStdDev() / 4, four.xyStdDev(), 1e-9)`.
- new `singleTagAtFourMeters`: `assertEquals(0.16, evaluate(pose(3, 4, 0), 1, 4.0, 0.05).xyStdDev(), 1e-9)`.

`VisionCameraTest`:
- new `multiTagCountsOnlyTheTagsInTheSolve`: a result with three targets (tags 9, 10, 11 from `robotPose()`) and a `MultiTargetPNPResult(new PnpResult(fieldToCamera, 0.1), List.of((short) 10, (short) 11))`, where `fieldToCamera = new Transform3d(Pose3d.ZERO, robotPose().transformBy(kRobotToCamera))`. Assert `stdDevs().get(0, 0)` equals `VisionFilter.evaluate(...)` for n = 2 and the mean distance of tags 10 and 11 only.
- new `stdDevFactorScalesTheTrust`: a camera with factor 2.0 gives twice the xy σ of the same frame on a camera with factor 1.0.

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `./gradlew test --tests 'frc.robot.subsystems.vision.*'`
Expected: FAIL: `moreTagsAreTrustedMore` (factor 2, not 4), `singleTagAtFourMeters` (0.32), and the two new camera tests (compile error on the 3-argument `CameraConfig`).

- [ ] **Step 3: Implement**

- `VisionFilter.evaluate`: n² in both σ, the new constants. Update the class doc and the tuning note.
- `VisionCamera`: for a multi-tag estimate, take `n` and the mean distance from the targets whose IDs are in `result.getMultiTagResult().get().fiducialIDsUsed`. Multiply both σ by `m_config.stdDevFactor()`.
- `Drive`, `DriveConstants`, `createDrivetrain` as in Interfaces. A doc line on the constants: small odometry σ means a vision frame moves the pose only a fraction of the way (6328 uses the same values).

- [ ] **Step 4: Run the tests and the smoke test**

Run: `./gradlew test` then `sh tools/sim-smoke.sh`
Expected: tests PASS; smoke `PASS`. In `build/sim-smoke.log`, end of step 7: error ≤ 0.05 m (the pull is now slower; step 7 lasts 7 s). If the error is larger, raise both floors to 0.03 and run again; record the value used in the commit message.

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems src/main/java/frc/robot/generated/TunerConstants.java src/test/java/frc/robot/subsystems/vision
git commit -m "fix(vision): set odometry trust and scale vision trust with 1/n^2 and per camera"
```

---

### Task 3: Heading from multi-tag frames, and a smoke check that fails

**Files:**
- Modify: `VisionFilter.java` (remove `kTrustVisionHeading`; `evaluate` gets a flag), `VisionConstants.java`, `VisionCamera.java`, `Vision.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java` (`getVision()`), `src/main/java/frc/robot/sim/SimSmokeTest.java` (`step`), `tools/sim-smoke.sh`
- Test: `VisionFilterTest.java`, `VisionCameraTest.java`

**Interfaces:**
- Consumes: Task 2 σ model.
- Produces:
  - `VisionFilter.evaluate(Observation obs, double fieldLength, double fieldWidth, boolean trustHeading)`. θ σ is finite only if `trustHeading && n >= 2 && d <= kMaxHeadingDistance` (`kMaxHeadingDistance = 4.0`); else `kUntrusted`.
  - `VisionConstants.kCamerasChecked = false`, with the doc text now on `kTrustVisionHeading` (how to check the cameras).
  - `VisionCamera(CameraConfig config, boolean trustHeading)`. `Vision` passes `RobotBase.isSimulation() || VisionConstants.kCamerasChecked`.
  - `int Vision.headingMeasurements()`: count of measurements sent with a finite θ σ since boot. `public Vision getVision()` in `RobotContainer`.

- [ ] **Step 1: Write the failing tests**

`VisionFilterTest` (call `evaluate(..., true)` in a new helper `evaluateTrusted`):
- replace `multiTagHeadingFollowsTheTrustSwitch` with:
  - `multiTagHeadingIsUsedWhenTrusted`: n = 3, d = 2 → `assertEquals(Math.max(0.02, 0.03 * 4 / 9), thetaStdDev, 1e-9)`.
  - `headingIsNotUsedUntilTheCamerasAreChecked`: same frame with `false` → θ σ ≥ 1e5.
  - `farMultiTagHeadingIsNotUsed`: n = 2, d = 4.5 → θ σ ≥ 1e5, still accepted.
- keep `singleTagHeadingIsNotTrusted` (now with `true`).

`VisionCameraTest`: construct with `false` in the existing tests; one new test `trustedMultiTagGivesAHeading` (the multi-tag frame of Task 2, camera built with `true`) asserts `stdDevs().get(2, 0) < 1`.

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `./gradlew test --tests 'frc.robot.subsystems.vision.*'`
Expected: FAIL, compile errors (4-argument `evaluate`, 2-argument `VisionCamera`).

- [ ] **Step 3: Implement the filter, the camera flag and the counter**

- [ ] **Step 4: Make the smoke test check the correction**

In `SimSmokeTest.step(...)`, right after `report("end of step " + m_step)`, when the step that ends is 7: print `SMOKE vision heading: %d measurements with a heading` (from `getVision().headingMeasurements()`), then, if the translation error is above 0.05 m or the absolute heading error (`truth.getRotation().minus(pose.getRotation())`, in degrees) is above 1.0, print a line that starts with `SMOKE FAIL: vision did not correct the kick`, followed by both errors. In `tools/sim-smoke.sh`, before `echo "PASS"`:
```sh
if grep -q "SMOKE FAIL" "$LOG"; then grep "SMOKE FAIL" "$LOG"; exit 1; fi
```

- [ ] **Step 5: Check that the smoke check can fail**

Temporarily set the sim trust to `false` in `Vision` (pass `VisionConstants.kCamerasChecked` only). Run `sh tools/sim-smoke.sh`.
Expected: exit 1 with `SMOKE FAIL: vision did not correct the kick` and a heading error of about 10 degrees. Undo the change.

- [ ] **Step 6: Run the tests and the smoke test**

Run: `./gradlew test` then `sh tools/sim-smoke.sh`, then `SIM_SMOKE_ALLIANCE=red sh tools/sim-smoke.sh`
Expected: tests PASS; both smoke runs `PASS`; `vision heading:` count > 0; end of step 7 heading within 1 degree of the truth. If the count is 0, the robot sees fewer than 2 tags at the step 6 spot: that is a test-setup problem, not a filter problem; report it, do not loosen `n >= 2`.

- [ ] **Step 7: Commit**

```bash
git add src/main/java/frc/robot src/test/java/frc/robot/subsystems/vision tools/sim-smoke.sh
git commit -m "feat(vision): heading from multi-tag frames (sim now, robot after the camera check); smoke test fails on an uncorrected kick"
```

---

### Task 4: Single-tag solution picked by the gyro heading

**Files:**
- Modify: `VisionFilter.java` (Observation, new reason, `closerToHeading`), `VisionCamera.java` (own single-tag path), `Vision.java`
- Test: `VisionFilterTest.java`, `VisionCameraTest.java`

**Interfaces:**
- Consumes: Tasks 1 to 3 signatures.
- Produces:
  - `VisionFilter.Observation(Pose3d pose, int tagCount, double averageDistance, double ambiguity, double headingError)`; `headingError` = |gyro heading − pose heading| in radians, 0 for multi-tag.
  - `VisionFilter.kMaxSingleTagHeadingError = Math.toRadians(15)`; a single-tag observation above it is rejected with reason `"heading"`. Multi-tag ignores `headingError`.
  - `static Pose3d VisionFilter.closerToHeading(Pose3d best, Pose3d alt, Rotation2d heading)`: the pose whose yaw is nearer `heading` (ties: `best`).
  - `VisionCamera.process(List<PhotonPipelineResult> results, double visionNowSeconds, Rotation2d heading)`. `Vision` passes `m_drive.getHeading()` read once per loop.

- [ ] **Step 1: Write the failing tests**

`VisionFilterTest` (update the helper to pass `headingError`, default 0):
- `closerToHeadingPicksTheMatchingSolution`: best yaw 40°, alt yaw 0°, heading 2° → returns alt.
- `rejectsSingleTagFarFromGyroHeading`: n = 1, headingError 20° → not accepted, reason `"heading"`.
- `multiTagIgnoresTheGyroHeading`: n = 3, headingError 30° → accepted (a Pigeon reboot must not block the fix).

`VisionCameraTest` (a helper `resultWith(best, alt, ambiguity)` builds the target from two robot poses: `robotPose()` and `flipped` = `robotPose()` moved 0.3 m sideways and turned +50°):
- `singleTagUsesTheSolutionThatMatchesTheGyro`: best from `flipped`, alt from `robotPose()`, ambiguity 0.1, heading = `robotPose()` yaw → the pose equals `robotPose().toPose2d()` within 1e-6.
- `singleTagFarFromBothSolutionsIsRejected`: heading = `robotPose()` yaw + 90° → empty, reason `"heading"`.
- The existing tests pass `robotPose().toPose2d().getRotation()` as the heading.

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `./gradlew test --tests 'frc.robot.subsystems.vision.*'`
Expected: FAIL (compile errors: 5-field `Observation`, `closerToHeading`, 3-argument `process`).

- [ ] **Step 3: Implement**

`VisionCamera`: replace `estimateLowestAmbiguityPose` with its own single-tag path. Take the fiducial target with the lowest ambiguity (skip ambiguity −1, as PhotonLib does), build both robot poses with `tagPose.transformBy(cameraToTarget.inverse()).transformBy(robotToCamera.inverse())` for `getBestCameraToTarget()` and `getAlternateCameraToTarget()`, pick with `closerToHeading`, fill `headingError`, then call `evaluate`. The timestamp is `result.getTimestampSeconds()`. Update the class doc ("the solution nearer the gyro heading", why).

- [ ] **Step 4: Run the tests and the smoke test**

Run: `./gradlew test` then `sh tools/sim-smoke.sh`
Expected: tests PASS; smoke `PASS` with the step 7 check.

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/vision src/test/java/frc/robot/subsystems/vision
git commit -m "fix(vision): pick the single-tag solution that matches the gyro, reject both when neither does"
```

---

## After the four tasks

- Push `nick-test` only when the user says so.
- On the robot, in this order: Task 1 step 8 (offsets), then the camera check from the review (roll, pitch, z per camera on the Vision tab), then set `kCamerasChecked = true` in its own commit.
