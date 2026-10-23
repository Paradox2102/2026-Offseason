# Shooting changes and calibration tool, implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Clear the turret entrance after a shot, pack and pulse the intake while shooting; then a
Test-mode calibration tool whose results live in a file on the robot.

**Spec:** `docs/specs/2026-10-22-shooting-qol-and-calibration-design.md`.

## Global Constraints

- Global constraints of `2026-10-22-sp1-foundation-drive.md` apply.
- Pure logic in small classes with JUnit tests; subsystems apply it.
- Push only to `nick-test`. No `simgui*.json`, no `.DS_Store` in commits.
- Gradle: `JAVA_HOME=~/wpilib/2027_alpha7/jdk bash ./gradlew test`.
- Sim: `JAVA_HOME=~/wpilib/2027_alpha7/jdk sh tools/sim-smoke.sh` with `SIM_SMOKE_AUTO`,
  `SIM_SMOKE_ALLIANCE`, `SIM_SMOKE_RACK`. Run matrices from bash, not zsh.

## Review Focus

1. **Trench:** no pack near a trench; `isExtended()` false while packing; the auto trench brake
   and the trench alert see the real state.
2. **Rack trust:** the position mode never sets a Talon position and never confirms a side.
3. **Auto intake event during a shot:** `extend()` must not hang while the rack packs.
4. **Calibration in a match:** nothing runs outside Test mode; a bad file falls back to constants.
5. **Shot table merge:** a recorded point changes the shot at once; the tables stay sorted.

---

## Sub-project 1

### Task 1: FeedLogic END

**Files:** `subsystems/shooter/FeedLogic.java`, `FeedLogicTest.java`, `ShooterConstants.java`.

- [ ] Tests first: end after a feed runs the last stage on, then all reverse for the clear time,
  then brake; end after a pause reverses at once; end with no feed since the last end brakes;
  end during an unjam reverses for the clear time; a pause stays as today.
- [ ] `Mode.END`; constructor takes `clearTime`; `isClearing(now)`.
- [ ] `kFeedClearTime = 0.5`.

### Task 2: Feeder

- [ ] `feedWhen` ends with `Mode.END`; `periodic` applies the logic while STOP or END; state
  "Clearing"; `isFeeding()` false while clearing.

### Task 3: PackPulse

**Files:** `subsystems/superstructure/PackPulse.java`, test.

- [ ] Tests: NaN before the delay; 0 offset during the settle time; sine after (amplitude at a
  quarter period, 0 at half); NaN for a NaN start.
- [ ] `PackPulse(delay, settleTime)`, `double offset(double sinceFeedStart, double amplitude, double rateHz)`.

### Task 4: IntakeRack position mode

- [ ] `Phase.POSITION`, `Side.hold(goal)`; `record Pack(double fraction, double offset)`;
  `packWhile(Supplier<Pack>)`; periodic: pack while enabled and the goal is out, else a side in
  POSITION goes back with `start(m_goalOut)`.
- [ ] `isExtended()` false while packing; `isPacking()`; `atLeast(fraction, tolerance)`;
  `extend()`/`retract()` end while packing.

### Task 5: IntakeRoller runInWhile

- [ ] `runInWhile(BooleanSupplier)`; periodic runs in when no command drives the rollers;
  brakes once when it ends.

### Task 6: Superstructure

- [ ] Feed start time (first feed of the shot), pack tunables, `packGoal()`, roller in while
  packing, the shot gate while packing; telemetry `Intake/Pack/...`.

### Task 7: Smoke test, all runs, spec, push

- [ ] Steps 13 and 15 print the pack and the clear.
- [ ] Runs: right, left, right-sotm, left-sotm, red, blocked. Spec 1.4. Commit, push.

## Sub-project 2

### Task 8: Pure classes

**Files:** `calibration/CalibrationData`, `CalibrationStore`, `ShotPoints`, `SteerOffsets`,
`DipCounter`, tests.

- [ ] Tests first for each (spec 2.4).

### Task 9: Hooks in the subsystems

- [ ] `ShotCalculator.setScoreTables`; `Turret` trim; `IntakeRack.setOutStops`
  (`RackSide.setOutStop`); `Hood.setCalibration(ratio, aboveLevelDeg)` and a jog; `Feeder`
  speeds and a single-stage run; steer offsets in `TunerConstants.createDrivetrain`;
  `Superstructure.shootWith(flywheel, hood)`.
- [ ] `Calibration.load()` at boot (before `Constants.Subsystems`).

### Task 10: Procedures and bindings

- [ ] `calibration/Calibration` (chooser, dashboard, save) and one class per procedure; the test
  pad bound once per procedure, active only in Test mode with that procedure picked.
- [ ] SysId moves into a procedure.

### Task 11: Elastic tab, sim script

- [ ] Calibration tab in `make-elastic-layout.py`; regenerate the layout.
- [ ] `sim/SimCalibrationTest`, `tools/sim-calibration.sh`.

### Task 12: Runs, spec, push

- [ ] Unit tests, smoke (right), calibration sim. Spec 2.5 and 2.6. Commit, push.
