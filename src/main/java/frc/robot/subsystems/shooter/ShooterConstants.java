package frc.robot.subsystems.shooter;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

/**
 * Shooter values. Turret angles are turret rotations (0 = the jig position, pointing to the
 * robot's right); hood angles are degrees above its lower stop; flywheel speeds are wheel rps;
 * feed speeds are motor rps.
 */
public final class ShooterConstants {
    private ShooterConstants() {}

    // ---- Turret (Talon 44, Kraken X60, CAN_S3) ----

    /** Motor rotations per turret rotation: 12:60, 14:28, 14:95 (CAD; same as the old code). */
    public static final double kTurretRatio = 67.86;
    /**
     * In the jig at power-on, the shooter points to the robot's right: turret 0 = robot -90
     * degrees (user, 2026-10-23: 180 degrees from the CAD direction).
     */
    public static final double kTurretZeroOnRobotDeg = -90;
    /** In the CAD (and the AdvantageScope model built from it), the shooter points to the robot's left. */
    public static final double kTurretCadOnRobotDeg = 90;
    /** Mechanical limits (user, 2026-10-23): 270 degrees each way from the jig, 540 degrees in all. */
    public static final double kTurretTravelRot = 0.75;
    /**
     * The Talon's soft limits: 10 degrees inside the mechanical limits, room for an overshoot or a
     * small zero error (a jig zero that is off, or a position restored after a reboot).
     */
    public static final double kTurretMinRot = -kTurretTravelRot + 10.0 / 360;
    public static final double kTurretMaxRot = kTurretTravelRot - 10.0 / 360;
    /**
     * The aim uses 5 degrees less each way, so the turret never presses its mechanical stop. At
     * the edge of this range, a target that keeps moving makes the turret unwind the other way
     * round (TurretMath picks the nearest angle that is inside the range).
     */
    public static final double kTurretAimMinRot = kTurretMinRot + 5.0 / 360;
    public static final double kTurretAimMaxRot = kTurretMaxRot - 5.0 / 360;
    /** Turret axis from the robot center, meters (CAD 2026-10-22: turret gear center (-6.0, +7.5) in). */
    public static final Translation2d kTurretOffset =
        new Translation2d(Units.inchesToMeters(-6.0), Units.inchesToMeters(7.5));
    /**
     * The turret goal leads the aim angle by its rate of change x this, seconds: the turret lags a
     * goal that keeps moving (a turning robot, or a robot driving past the target). Sim: tuned in
     * the shoot-on-the-move test. Robot: measure the turret's lag while it tracks.
     */
    public static final double kTurretLeadTime = 0.05;
    /** The hood and flywheel goals are for the distance this far ahead (they lag a shot whose distance keeps changing), seconds. */
    public static final double kHoodLeadTime = 0.1;
    public static final double kFlywheelLeadTime = 0.08;
    /**
     * From the shot being worked out to the ball leaving, seconds: the shot is aimed from where
     * the robot will be then. Sim: one loop (the sim shoots in the loop after the feed starts).
     * Robot: the loop, plus the time a ball takes from the feeder to the wheel; measure it.
     */
    public static final double kShotLatency = 0.02;
    /**
     * The speed a ball takes from the robot dies away in the air (drag), so it drifts less than
     * velocity x tof x (1 - this x tof), per second. In theory half the decay rate of the
     * ball's speed (drag 0.0216 1/m x ~9 m/s = 0.19 per second, so about 0.1). Sim: 0.10 from the
     * shoot-on-the-move test (0 left the balls behind the robot's motion, 0.19 ahead of it).
     * Robot: tune with test shots on the move.
     */
    public static final double kLeadDecay = 0.10;
    /**
     * Shots go only while the robot is slower than this (m/s) and turns slower than this (rad/s).
     * Sim (tools/sim-sotm.sh): every ball shot scored up to 3 m/s, mean miss 0.06-0.10 m strafing
     * and 0.23 m on an arc at 3 m/s; spinning at 2 rad/s 0.16 m, 0.36 m with a 2 m/s strafe as
     * well (the opening is 0.53 m in radius, 0.45 m for the ball's edge). With shoot on the move
     * off on the dashboard, the still limits apply.
     */
    public static final double kMaxShootSpeed = 3.2;
    public static final double kMaxShootTurnRate = 2.2;
    public static final double kStillShootSpeed = 0.25;
    /**
     * Shots go only while the aim distance (to the aim point, after the lead) is inside the
     * shot tables: driving fast at the hub moves the aim point to between the robot and the hub,
     * and driving fast away moves it past the tables' end.
     */
    public static final double kMinShotDistance = 1.25;
    public static final double kMaxShotDistance = 6.0;
    public static final double kStillShootTurnRate = 0.3;
    /** Torque-current gains (amps). Start values: tune kP until the turret holds without shaking. */
    public static final double kTurretS = 2.0; // A
    public static final double kTurretP = 360; // A per turret rotation (1 A per degree)
    public static final double kTurretD = 15; // A per turret rps
    /**
     * Motion Magic Expo profile. The plan's top speed is 12 V / kV and its acceleration 12 V / kA.
     * 0.12 V per motor rps x 67.86 = 8.1 would plan at the motor's full speed, where the back
     * voltage leaves little room to brake. 16 plans at half of that (0.75 turret rps, 270 deg/s),
     * and kA 3 at 4 turret rps per second. Raise both if the turret is too slow on the robot.
     */
    public static final double kTurretExpoV = 16; // V per turret rps
    public static final double kTurretExpoA = 3; // V per turret rps^2
    public static final double kTurretStatorLimit = 40; // A
    /** On target within this angle. */
    public static final double kTurretToleranceDeg = 1.5;
    /**
     * For a shot, the turret is on target when its error would move the ball less than this
     * sideways at the hub, meters (the opening is 0.53 m in radius), but at least
     * kTurretMinToleranceDeg: 4.3 degrees at 2 m, 1.9 at 4.5 m.
     */
    public static final double kTurretToleranceMeters = 0.15;
    public static final double kTurretMinToleranceDeg = 1.0;
    /**
     * The turret cannot turn more than this in one 20 ms loop, rotations (270 deg/s is 5.4
     * degrees a loop). A bigger jump in its reading is a reboot (it reads 0 again), not a move.
     */
    public static final double kTurretMaxStepRot = 15.0 / 360;

    // ---- Hood (Talon 41, Kraken X44, CAN_S3) ----

    /**
     * Motor rotations per hood rotation. ESTIMATE from the CAD: MAXPlanetary 5:1, then a 9T pinion
     * (11.4 mm pitch radius) on a sector about 195 mm from the hood pivot: 5 x 17 = 85. Measure:
     * move the hood from its lower to its upper stop by hand, read the motor rotations in Tuner X,
     * divide by the hood's travel in rotations.
     */
    public static final double kHoodRatio = 85;
    /**
     * Highest hood angle to ask for, degrees. Measured 2026-10-23: the hood's upper stop is 8.84
     * motor turns from the lower stop, 37.4 degrees at 85:1 (was 40, past the stop). The Talon
     * soft limit is 1 degree above this.
     */
    public static final double kHoodMaxDeg = 36;
    /** Voltage gains, per hood rotation. kP 180 V/rot = 0.5 V per degree. */
    public static final double kHoodS = 0.2;
    public static final double kHoodP = 180;
    /** Gravity on the hood: 0 until the hood's angle above level at its stop is measured. */
    public static final double kHoodG = 0;
    /** The hood's angle above level when it sits on its lower stop, rotations. Measure, then set kHoodG. */
    public static final double kHoodStopAboveLevelRot = 0;
    public static final double kHoodCruise = 0.5; // hood rps
    public static final double kHoodAcceleration = 3; // hood rps^2
    public static final double kHoodStatorLimit = 30; // A
    /**
     * Find zero: drive down into the lower stop at this voltage. A voltage caps the speed (about
     * 40 deg/s at 85:1, with gravity helping a little more), where a current push sped up until it
     * hit the stop at ~500 deg/s. Stalled, 1 V still gives ~20 A, enough to break friction, so a
     * stall is the stop and not a sticky hood. Check on the robot: the hood must reach the stop
     * from its top in about 1 s.
     */
    public static final double kHoodHomeVolts = -1.0;
    public static final double kHoodToleranceDeg = 1.0;

    // ---- Flywheel (Talons 42 + 43, Kraken X60, CAN_S3) ----

    /** Motor rotations per wheel rotation: 14:14, then 40:22. The wheel turns faster than the motor. */
    public static final double kFlywheelRatio = 0.55;
    public static final double kFlywheelS = 3; // A
    public static final double kFlywheelP = 5; // A per wheel rps of error
    public static final double kFlywheelStatorLimit = 80; // A
    public static final double kFlywheelSupplyLimit = 37; // A (old code)
    /** At speed within this fraction of the target, for kFlywheelReadyTime. */
    public static final double kFlywheelTolerance = 0.03;
    public static final double kFlywheelReadyTime = 0.1;
    /** Moment of inertia of the flywheel, kg m^2, for the sim only (old code). */
    public static final double kFlywheelMoi = 0.0004812;

    // ---- Feed path, motor rps ----

    /*
     * Speeds and directions: what main (8fd2c1a, 2026-10-25) shoots with, found on the robot by
     * the students (commits 0f1d92a "Inverts" and cf364a5). Main feeds with: indexer roller 30 at
     * full duty cycle (here as a speed: 12 V / kV), top belt 31 at 90 rps with bottom belt 32 as an
     * Aligned follower (same turn direction, same speed), Scotty 45 at 90 rps, vertical roller 40
     * at 90 rps. Main sets no mechanism ratio on these, so all are motor rps. The inversions in
     * Feeder make each of these speeds positive.
     *
     * Control: each stage is a Motion Magic velocity loop (MotionMagicVelocityVoltage, no FOC:
     * these five Talons have no Pro license). Motion Magic ramps the speed at kFeedAcceleration,
     * so a start does not pull a current spike; kS + kV x speed gives about the right volts, and kP
     * adds volts when a ball slows the stage. kS, kV, kP: main's tuned values for 31/32 (kI not
     * used), 45 and 40. The indexer roller's gains on main are amps (torque current), so its kS
     * and kP here are start values. Units: V, V per motor rps, V per motor rps of error.
     *
     * Main runs the indexer roller at full duty cycle. As a speed that is 12 V / kV = 96 rps: the
     * feedforward alone then asks the whole battery, kP has nothing left, and the stage is straight
     * power again. 85 rps keeps every stage at or under kFeedMaxFeedforwardVolts.
     */
    public static final double kIndexerRollerKs = 0.25; // Talon 30, X60, CAN_S1
    public static final double kIndexerRollerKv = 0.125;
    public static final double kIndexerRollerKp = 0.1;
    public static final double kIndexerRollerRps = 85;
    public static final double kBeltTunnelKs = 0.0; // Talons 31 and 32, X44, CAN_S1
    public static final double kBeltTunnelKv = 0.097;
    public static final double kBeltTunnelKp = 0.05;
    public static final double kTopBeltRps = 90;
    public static final double kBottomBeltRps = 90;
    public static final double kScottyKs = 0.6; // Talon 45 (code name "goUp"), X60, CAN_S3
    public static final double kScottyKv = 0.115;
    public static final double kScottyKp = 0.06;
    public static final double kScottyRps = 90;
    public static final double kVerticalRollerKs = 0.25; // Talon 40 (code name "kicker"), X60, CAN_S3
    public static final double kVerticalRollerKv = 0.118;
    public static final double kVerticalRollerKp = 0.09;
    public static final double kVerticalRollerRps = 90;
    /**
     * The most volts a stage's feedforward can ask at its feed speed. The rest of the battery
     * (it sags to about 11 V while the flywheel spins up) is for kP.
     */
    public static final double kFeedMaxFeedforwardVolts = 11.0;
    /**
     * Motor rps per second: 90 rps in 0.075 s. An unjam goes from +90 to -45 rps in 0.11 s, so
     * most of its 0.3 s really runs back (at 600 it took 0.23 s to get there).
     */
    public static final double kFeedAcceleration = 1200;
    /** Stator limits, amps. Start values: measure each stage's current, empty and loaded. */
    public static final double kFeedStatorLimitX60 = 60;
    public static final double kFeedStatorLimitX44 = 40;
    /** A stage stalled: slower than this fraction of its speed with more than this fraction of its stator limit ... */
    public static final double kFeedJamSpeedFraction = 0.2;
    public static final double kFeedJamCurrentFraction = 0.75;
    /** ... for this long is a jam, seconds; then every stage runs back for kFeedUnjamTime. */
    public static final double kFeedJamTime = 0.25;
    public static final double kFeedUnjamTime = 0.3;
    /** Alert after this many jams within kFeedJamWindow seconds. */
    public static final int kFeedJamAlertCount = 3;
    public static final double kFeedJamWindow = 5.0;
    /** When the feed stops, the vertical roller (40) runs on this long, so it does not hold a ball against the wheel. */
    public static final double kFeedRunOnTime = 0.25;
    /**
     * At the end of a shot (not a pause), after the run-on, every stage runs back this long, so no
     * ball is left at the turret entrance. Check on the robot: no ball left there, none pushed
     * back out of the hopper.
     */
    public static final double kFeedClearTime = 0.5;
    /** Eject: every feed motor runs out at this fraction of its feed speed. */
    public static final double kEjectFraction = -0.5;

    // ---- Shot tables: {distance m, value} ----

    /**
     * Flywheel speed, wheel rps, by distance from the turret axis to the hub center. SOLVED IN
     * THE SIM: the speed at which a still shot (with the hood table below) comes down through the
     * middle of the hub opening in the sim's ball model (sim/field/ShotModel; ShotTablesTest
     * checks it). That model's exit speed is fitted, not measured: measure these on the robot.
     * The old code's speeds, in MOTOR rps (x 1 / 0.55 for wheel rps), for reference:
     * {1.125, 40}, {2.25, 45}, {2.75, 47}, {3.25, 50}, {4, 55}, {4.75, 60}.
     */
    public static final ShotTable kScoreSpeeds = new ShotTable(new double[][] {
        {1.25, 78.8}, {1.5, 78.05}, {1.75, 80.56}, {2.0, 82.71}, {2.25, 85.11},
        {2.5, 87.53}, {2.75, 90.03}, {3.0, 92.45}, {3.25, 94.71}, {3.5, 97.08},
        {3.75, 99.51}, {4.0, 101.62}, {4.25, 104.04}, {4.5, 106.31}, {4.75, 108.66},
        {5.0, 110.82}, {5.25, 113.11}, {5.5, 115.18}, {5.75, 117.31}, {6.0, 119.49}});
    public static final ShotTable kPassSpeeds = kScoreSpeeds;
    /**
     * Hood angles, degrees above the lower stop. BEST GUESS, to tune on the robot: each shot comes
     * down into the hub at about 45 degrees (steep enough to drop in over the near rim, and less
     * long/short error than a flat shot), with launch = 78 - hood (ShotModel's fit, not measured).
     * Launch angle for a 45 degree entry with no drag: tan(launch) = 1 + 2 x rise / run, rise 1.37 m
     * (hub top 1.83 m less the exit 0.455 m), run = distance - 0.084 m. At 1.25 m that hits the
     * rim in the sim, so it is 2 (the steepest shot that comes down on the center). Rounded to
     * 0.5 degree. The 10 m point is for passes only (kept from the old table). Were the old
     * code's numbers, which were motor degrees there (1.5 m: 6, 2 m: 15, 3-4 m: 22, 5 m: 25).
     */
    public static final ShotTable kScoreHood = new ShotTable(new double[][] {
        {1.25, 2.0}, {1.5, 7.0}, {1.75, 8.5}, {2.0, 10.5}, {2.25, 12.0},
        {2.5, 13.0}, {2.75, 14.0}, {3.0, 15.0}, {3.25, 16.0}, {3.5, 17.0},
        {3.75, 17.5}, {4.0, 18.5}, {4.25, 19.0}, {4.5, 19.5}, {4.75, 20.0},
        {5.0, 20.5}, {5.25, 21.0}, {5.5, 21.5}, {5.75, 22.0}, {6.0, 22.5}, {10, 35}});
    public static final ShotTable kPassHood = kScoreHood;
    /**
     * Time of flight, seconds: from the shot until the ball comes down through the height of the
     * hub top. From the same sim model as kScoreSpeeds. Shoot on the move aims ahead by the
     * robot's velocity x this. Measure on the robot (video at 240 fps from the side).
     */
    public static final ShotTable kScoreTimeOfFlight = new ShotTable(new double[][] {
        {1.25, 0.825}, {1.5, 0.755}, {1.75, 0.795}, {2.0, 0.82}, {2.25, 0.85},
        {2.5, 0.89}, {2.75, 0.92}, {3.0, 0.955}, {3.25, 0.975}, {3.5, 1.0},
        {3.75, 1.035}, {4.0, 1.055}, {4.25, 1.085}, {4.5, 1.11}, {4.75, 1.135},
        {5.0, 1.16}, {5.25, 1.185}, {5.5, 1.2}, {5.75, 1.225}, {6.0, 1.245}});
    public static final ShotTable kPassTimeOfFlight = kScoreTimeOfFlight;
}
