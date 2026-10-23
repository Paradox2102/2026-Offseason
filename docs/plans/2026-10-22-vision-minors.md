# Vision minors M1 to M8: implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the eight Minor findings that the `vision-trust` review deferred, each with a test.

**Architecture:** The vision code stays as it is: pure decisions in `VisionFilter`, `VisionClock` and `HeadingReference`; `VisionCamera` turns frames into measurements; `Vision` reads the clocks and the heading and feeds the Phoenix estimator. The fixes go into those same classes. Only M5 (clock reads in `Vision`) has no unit test, because `Vision` needs hardware; the sim smoke test covers it.

**Tech Stack:** Java 21, WPILib 2027.0.0-alpha-7 (`org.wpilib.*`), Phoenix 6 26.70.0-alpha-2, PhotonLib dev-v2027.0.0-alpha-2-80-g82f7f4c3, JUnit 5.

**Spec:** `vision-minors-spec.md` in the shared notes folder (`~/Library/Application Support/Claude/side-session-notes/local_0e5bc3f2-c9d7-4ae0-b883-1d84324e99ee/`). Background: `docs/plans/2026-10-22-vision-heading-and-trust.md` and the ledger `vision-trust-ledger.md` in the same notes folder.

## Global Constraints

- Start from the tip of `nick-test` (a6af007 or later) in a worktree under `.claude/worktrees/` (it is in `.git/info/exclude`). Another session commits on `nick-test` in the main checkout: find code by name, not by line.
- Build and test with `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh ./gradlew test --rerun` (`gradlew` is not executable in a worktree; without `--rerun`, Gradle can skip the tests).
- The last task runs `sh tools/sim-smoke.sh` and `SIM_SMOKE_ALLIANCE=red sh tools/sim-smoke.sh`: both must print `PASS`.
- Commit messages: Conventional Commits, no em dash (a hook rejects it), no Claude attribution.
- Comments: plain short sentences, the style of `VisionFilter.java`.
- Values: time-sync limit 2.0 s (`timeSinceLastPong` is in nanoseconds; PhotonLib warns at 5 s; the sim reports 1 ms). Assumed: a PhotonVision coprocessor gets a pong at least once a second. Display hold 0.5 s. "No alternate" means a translation norm below 1e-6 m.

## Review Focus

- A multi-tag result that falls back to the single-tag path must meet the single-tag gates (ambiguity, gyro), and say why it was rejected (Task 1 test `fallbackFrameUsesTheSingleTagGates`).
- A factor of NaN in a `CameraConfig` must fail at boot, like 0 (Task 2 test `cameraConfigRejectsABadFactor`).
- The sim must never trip the time-sync gate: the sim reports a pong 1 ms ago (Task 3 step: no "time sync" reason in the smoke log).
- A camera that stops sending: its latency must show NaN after 0.5 s, not its last healthy value (Task 4 test `latencyGoesNaNWhenFramesStop`).
- A frame rejected for age must not hide a newer good frame of the same batch (Task 5 test `oldFrameDoesNotBlockANewOneInTheSameBatch`).

---

### Task 1: Solve fixes: fallback, zero alternate, distance (M1, M3, M7 part 1)

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/vision/VisionCamera.java` (`multiTagSolve`, `singleTagSolve`, the `Solve` record, the distance in `process`)
- Test: `src/test/java/frc/robot/subsystems/vision/VisionCameraTest.java`

**Interfaces:**
- Produces: `private record Solve(Pose3d pose, List<PhotonTrackedTarget> used, double ambiguity, double headingError, boolean multiTag, double distance)`. `process` uses `s.distance()` and no longer computes it.
- Test helpers (in `VisionCameraTest`): `multiTagAt(double seconds, List<Short> used, double ambiguity)` (the 2-argument version passes 0.05); `resultWithAlt(Pose3d bestRobot, Transform3d altCameraToTag)`.

- [ ] **Step 1: Write the failing tests**

```java
@Test void multiTagWithOneUsedTagFallsBackToSingleTag() {
    var m = m_camera.process(List.of(multiTagAt(10.0, List.of((short) 10))), 10.03, t -> heading());
    assertEquals(1, m.size());
    assertFalse(m.get(0).multiTag());
    assertEquals(1, m_camera.lastTagCount());
}
@Test void multiTagWithNoUsedTagFallsBackToSingleTag() {
    var m = m_camera.process(List.of(multiTagAt(10.0, List.of())), 10.03, t -> heading());
    assertEquals(1, m.size());
    assertFalse(m.get(0).multiTag());
}
@Test void fallbackFrameUsesTheSingleTagGates() {
    assertTrue(m_camera.process(List.of(multiTagAt(10.0, List.of((short) 10), 0.5)), 10.03, t -> heading()).isEmpty());
    assertEquals("ambiguity", m_camera.lastRejectReason());
}
@Test void zeroAlternateIsNeverPicked() {
    // A zero alternate would put the robot at the tag; make the gyro agree with that pose.
    var atTag = FieldConstants.kField.getTagPose(kTag).orElseThrow().transformBy(kRobotToCamera.inverse());
    m_camera.process(List.of(resultWithAlt(robotPose(), new Transform3d())), 10.03, t -> atTag.toPose2d().getRotation());
    assertEquals(0, robotPose().getTranslation().getDistance(m_camera.lastEstimate().orElseThrow().getTranslation()), 1e-6);
}
@Test void singleTagDistanceIsFromThePickedSolution() {
    var best = robotPose().transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.toRadians(10))));
    var alt = robotPose().transformBy(new Transform3d(new Translation3d(1.5, 0, 0), new Rotation3d()));
    var m = m_camera.process(List.of(resultWith(best, alt, 0.1)), 10.03, t -> heading());
    double distance = cameraToTagFrom(alt).getTranslation().getNorm();
    var expected = VisionFilter.evaluate(new VisionFilter.Observation(alt, 1, distance, 0.1, 0),
        FieldConstants.kFieldLength, FieldConstants.kFieldWidth, false);
    assertEquals(expected.xyStdDev(), m.get(0).stdDevs().get(0, 0), 1e-9);
}
```

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh ./gradlew test --rerun --tests 'frc.robot.subsystems.vision.*'`
Expected: compile error on `multiTagAt(..., 0.5)` and `resultWithAlt` until the helpers exist. With the helpers: the three fallback tests FAIL (rejected as "no tags", or accepted as tag count 1 with no ambiguity gate); `zeroAlternateIsNeverPicked` FAILS (the estimate sits at the tag); `singleTagDistanceIsFromThePickedSolution` FAILS (σ from the best transform's distance).

- [ ] **Step 3: Implement**

- `multiTagSolve`: return `Optional.empty()` when fewer than 2 listed targets are in `fiducialIDsUsed`; `process` then tries `singleTagSolve`. Distance = the mean best-transform norm of `used`.
- `singleTagSolve`: when the alternate transform's translation norm is below 1e-6, use the best pose only (no `closerToHeading`). Distance = the norm of the transform of the picked solution.

- [ ] **Step 4: Run the tests**

Run: `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh ./gradlew test --rerun`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add docs/plans/2026-10-22-vision-minors.md src/main/java/frc/robot/subsystems/vision/VisionCamera.java src/test/java/frc/robot/subsystems/vision/VisionCameraTest.java
git commit -m "fix(vision): fall back to one tag when the multi-tag solve used fewer than 2, ignore a zero alternate, distance from the picked solution"
```

---

### Task 2: Camera trust factor (M2)

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/vision/VisionConstants.java` (`CameraConfig`), `VisionCamera.java` (the σ of each `Measurement`)
- Test: `VisionCameraTest.java`

**Interfaces:**
- Produces: `CameraConfig` compact constructor throws `IllegalArgumentException("stdDevFactor must be > 0 for camera " + name)` unless `stdDevFactor > 0` (NaN fails too). θ σ is multiplied by the factor only when it is below `VisionFilter.kUntrusted`; otherwise it stays exactly `kUntrusted`.

- [ ] **Step 1: Write the failing tests**

```java
@Test void cameraConfigRejectsABadFactor() {
    assertThrows(IllegalArgumentException.class, () -> new VisionConstants.CameraConfig("c", kRobotToCamera, 0));
    assertThrows(IllegalArgumentException.class, () -> new VisionConstants.CameraConfig("c", kRobotToCamera, Double.NaN));
}
@Test void untrustedHeadingIsNotScaled() {
    var m = camera(2.0, false).process(List.of(resultAt(10.0, 0.05)), 10.03, t -> heading());
    assertEquals(VisionFilter.kUntrusted, m.get(0).stdDevs().get(2, 0), 0);
}
```

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh ./gradlew test --rerun --tests 'frc.robot.subsystems.vision.*'`
Expected: FAIL: no exception thrown; θ is 2e6.

- [ ] **Step 3: Implement** (as in Interfaces; say in the record doc that a bad factor stops the robot code at boot, on purpose).

- [ ] **Step 4: Run the tests**

Run: `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh ./gradlew test --rerun`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/vision src/test/java/frc/robot/subsystems/vision
git commit -m "fix(vision): reject a camera trust factor that is not above 0, keep an untrusted heading untrusted"
```

---

### Task 3: Time-sync gate and clocks read per camera (M4, M5)

**Files:**
- Modify: `VisionClock.java`, `VisionCamera.java` (`process`), `Vision.java` (`periodic`)
- Test: `VisionClockTest.java`, `VisionCameraTest.java`

**Interfaces:**
- Produces: `VisionClock.kMaxTimeSinceSync = 2.0` (seconds) and `static String syncRejectReason(long timeSinceLastPongNanos)`: `"time sync"` above the limit, else `""`. `VisionCamera.process` checks it right after the age check, with `result.metadata.timeSinceLastPong`.
- Test helper: `metadataAt(double seconds, long timeSinceLastPongNanos)` (the 1-argument version passes 0).

- [ ] **Step 1: Write the failing tests**

`VisionClockTest`:
```java
@Test void acceptsARecentPong() { assertEquals("", VisionClock.syncRejectReason(1_000_000L)); }
@Test void rejectsAnOldPong() { assertEquals("time sync", VisionClock.syncRejectReason(3_000_000_000L)); }
```
`VisionCameraTest`:
```java
@Test void frameWithoutTimeSyncIsRejected() {
    var result = new PhotonPipelineResult(metadataAt(10.0, 3_000_000_000L), List.of(target(kTag, 0.05)), Optional.empty());
    assertTrue(m_camera.process(List.of(result), 10.03, t -> heading()).isEmpty());
    assertEquals("time sync", m_camera.lastRejectReason());
}
```

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh ./gradlew test --rerun --tests 'frc.robot.subsystems.vision.*'`
Expected: FAIL (compile error: `syncRejectReason` does not exist).

- [ ] **Step 3: Implement the gate**

- [ ] **Step 4: Read the clocks per camera in `Vision.periodic` (M5)**

Move the `visionNow`/`driveNow` reads and the `estimatorAt`/`headingAt` functions into the camera loop, right after `readNewFrames()`, so each camera's frames are judged against a "now" read after they arrived. Keep the two offset logs once per loop, from a read at the top. No unit test: `Vision` needs hardware; Step 6 covers it.

- [ ] **Step 5: Run the tests**

Run: `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh ./gradlew test --rerun`
Expected: all PASS.

- [ ] **Step 6: Run the smoke test**

Run: `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh tools/sim-smoke.sh`
Expected: `PASS`; the step 7 check passes; `grep -c '"time sync"' build/sim-smoke.log` gives 0 (Review Focus: the sim never trips the gate).

- [ ] **Step 7: Commit**

```bash
git add src/main/java/frc/robot/subsystems/vision src/test/java/frc/robot/subsystems/vision
git commit -m "fix(vision): reject frames from a coprocessor without time sync, read the clocks after each camera's frames"
```

---

### Task 4: Held telemetry for latency and reject reason (M6)

**Files:**
- Modify: `VisionCamera.java`, `Vision.java` (`log`)
- Test: `VisionCameraTest.java`

**Interfaces:**
- Produces: `VisionCamera.kDisplayHoldSeconds = 0.5`;
  `double displayLatencySeconds(double visionNowSeconds)`: the newest frame's age from the last call that had frames, or NaN when that call was more than 0.5 s before `visionNowSeconds` (or never);
  `String displayRejectReason(double visionNowSeconds)`: the last non-empty reject reason, or `""` when it was set more than 0.5 s before.
  `lastLatencySeconds()` and `lastRejectReason()` keep their per-call meaning (existing tests).
- `Vision.log(VisionCamera camera, int frameCount, double visionNow)` (the per-camera `visionNow` from Task 3) logs `Latency ms` = `displayLatencySeconds × 1000` every loop (NaN included) and `Reject Reason` = `displayRejectReason`.

- [ ] **Step 1: Write the failing tests**

```java
@Test void latencyIsHeldBetweenFrames() {
    m_camera.process(List.of(resultAt(10.0, 0.05)), 10.02, t -> heading());
    m_camera.process(List.of(), 10.04, t -> heading());
    assertEquals(0.02, m_camera.displayLatencySeconds(10.04), 1e-6);
}
@Test void latencyGoesNaNWhenFramesStop() {
    m_camera.process(List.of(resultAt(10.0, 0.05)), 10.02, t -> heading());
    m_camera.process(List.of(), 10.6, t -> heading());
    assertTrue(Double.isNaN(m_camera.displayLatencySeconds(10.6)));
}
@Test void rejectReasonIsHeldBetweenFrames() {
    m_camera.process(List.of(resultAt(10.0, 0.6)), 10.03, t -> heading());
    m_camera.process(List.of(), 10.05, t -> heading());
    assertEquals("ambiguity", m_camera.displayRejectReason(10.05));
    assertEquals("", m_camera.displayRejectReason(10.6));
}
```

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh ./gradlew test --rerun --tests 'frc.robot.subsystems.vision.*'`
Expected: FAIL (compile error: the display methods do not exist).

- [ ] **Step 3: Implement** (the call time is `visionNowSeconds` of `process`).

- [ ] **Step 4: Run the tests**

Run: `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh ./gradlew test --rerun`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/vision src/test/java/frc/robot/subsystems/vision
git commit -m "fix(vision): hold latency and reject reason on the dashboard, NaN latency once a camera stops"
```

---

### Task 5: Missing tests and the Drive doc (M8, M7 part 2)

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/Drive.java` (constructor doc only)
- Test: `VisionCameraTest.java`

These tests cover behavior that exists, so they pass at once. Each one gets a mutation check: break the code on purpose, see the test fail, undo.

- [ ] **Step 1: Write the tests**

```java
@Test void futureFrameIsRejected() {
    assertTrue(m_camera.process(List.of(resultAt(10.0, 0.05)), 9.9, t -> heading()).isEmpty());
    assertEquals("future", m_camera.lastRejectReason());
}
@Test void multiTagIsAcceptedWithTheGyroFarOff() {
    var gyro = heading().plus(Rotation2d.fromDegrees(40)); // after a Pigeon reboot
    var m = m_camera.process(List.of(multiTagAt(10.0, List.of((short) 10, (short) 11))), 10.03, t -> gyro);
    assertEquals(1, m.size());
    assertTrue(m.get(0).multiTag());
}
@Test void oldFrameDoesNotBlockANewOneInTheSameBatch() {
    var m = m_camera.process(List.of(resultAt(9.0, 0.05), resultAt(10.0, 0.05)), 10.03, t -> heading());
    assertEquals(1, m.size());
    assertEquals(10.0, m.get(0).timestamp(), 1e-6);
}
```

- [ ] **Step 2: Run them**

Run: `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh ./gradlew test --rerun --tests 'frc.robot.subsystems.vision.*'`
Expected: PASS.

- [ ] **Step 3: Mutation checks (one at a time, undo each)**

- In `VisionCamera.process`, delete the age check: `futureFrameIsRejected` FAILS.
- In `VisionCamera.process`, try `singleTagSolve` before `multiTagSolve`: `multiTagIsAcceptedWithTheGyroFarOff` FAILS (the single-tag path rejects it as "heading").
- In `VisionCamera.process`, `return measurements` in place of `continue` after an age reject: `oldFrameDoesNotBlockANewOneInTheSameBatch` FAILS.

- [ ] **Step 4: Fix the Drive doc**

`@param odometryHz`: "odometry thread rate; 0 = the Phoenix default (250 Hz on CAN FD, 100 Hz on CAN 2.0; the SystemCore has CAN 2.0)".

- [ ] **Step 5: Run the full suite and both smoke runs**

Run: `JAVA_HOME=/Users/nickfoote/wpilib/2027_alpha7/jdk sh ./gradlew test --rerun`, then `sh tools/sim-smoke.sh`, then `SIM_SMOKE_ALLIANCE=red sh tools/sim-smoke.sh` (same `JAVA_HOME`).
Expected: all tests PASS; both smoke runs print `PASS`.

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/Drive.java src/test/java/frc/robot/subsystems/vision/VisionCameraTest.java
git commit -m "test(vision): future frame, multi-tag with the gyro far off, mixed-age batch; Drive doc names both CAN rates"
```
