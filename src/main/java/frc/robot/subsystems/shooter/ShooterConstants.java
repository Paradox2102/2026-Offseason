package frc.robot.subsystems.shooter;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

/**
 * Shooter values. Turret angles are turret rotations (0 = the jig position, pointing to the
 * robot's left); hood angles are degrees above its lower stop; flywheel speeds are wheel rps;
 * feed speeds are motor rps.
 */
public final class ShooterConstants {
    private ShooterConstants() {}

    // ---- Turret (Talon 44, Kraken X60, CAN_S3) ----

    /** Motor rotations per turret rotation: 12:60, 14:28, 14:95 (CAD; same as the old code). */
    public static final double kTurretRatio = 67.86;
    /** In the jig at power-on, the shooter points to the robot's left: turret 0 = robot +90 degrees. */
    public static final double kTurretZeroOnRobotDeg = 90;
    /** Wiring limits (user, 2026-10-22): one full turn each way from the jig, 720 degrees in all. */
    public static final double kTurretWiringRot = 1.0;
    /**
     * The Talon's soft limits: 10 degrees inside the wiring limits, room for an overshoot or a
     * small zero error (a jig zero that is off, or a position restored after a reboot).
     */
    public static final double kTurretMinRot = -kTurretWiringRot + 10.0 / 360;
    public static final double kTurretMaxRot = kTurretWiringRot - 10.0 / 360;
    /**
     * The aim uses 5 degrees less each way, so the turret never presses the wiring at its end. At
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
    /** Highest hood angle to ask for, degrees (old sim: 42). Check on the robot. */
    public static final double kHoodMaxDeg = 40;
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
     * Speeds: what the code that ran at CAEL (7ba98388, testRun) gave each stage as duty cycle,
     * as a speed: duty x 12 V / kV (the stage's speed at that duty with no load). The velocity
     * loop then runs each stage at about the CAEL speed whatever the battery does. Measure the
     * fire rate on the robot and tune from here. kV: V per motor rps (old code; X44 12 V / 125 rps).
     */
    public static final double kIndexerRollerKv = 0.125; // Talon 30, X60, CAN_S1
    public static final double kIndexerRollerRps = 1.0 * 12 / kIndexerRollerKv;
    public static final double kBeltTunnelKv = 0.096; // Talons 31 and 32, X44, CAN_S1
    public static final double kTopBeltRps = 1.0 * 12 / kBeltTunnelKv;
    public static final double kBottomBeltRps = 0.9 * 12 / kBeltTunnelKv;
    public static final double kScottyKv = 0.1195; // Talon 45 (code name "goUp"), X60, CAN_S3
    public static final double kScottyRps = 1.0 * 12 / kScottyKv;
    public static final double kVerticalRollerKv = 0.118; // Talon 40 (code name "kicker"), X60, CAN_S3
    public static final double kVerticalRollerRps = 0.9 * 12 / kVerticalRollerKv;
    /** Motor rps per second: the fastest stage (125 rps) is at speed in about 0.2 s. */
    public static final double kFeedAcceleration = 600;
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
        {1.25, 82.42}, {1.5, 78.73}, {1.75, 78.97}, {2.0, 81.2}, {2.25, 83.54},
        {2.5, 85.99}, {2.75, 88.38}, {3.0, 90.76}, {3.25, 93.15}, {3.5, 95.48},
        {3.75, 97.96}, {4.0, 100.35}, {4.25, 102.72}, {4.5, 105.02}, {4.75, 107.28},
        {5.0, 109.41}, {5.25, 111.79}, {5.5, 114.03}, {5.75, 116.25}, {6.0, 118.28}});
    public static final ShotTable kPassSpeeds = kScoreSpeeds;
    /** Hood angles, degrees: the old code's (motor degrees there, not measured as hood degrees), but 6 at 1.5 m (10 hit the rim in the sim). */
    public static final ShotTable kScoreHood = new ShotTable(new double[][] {
        {1.125, 0}, {1.25, 0}, {1.5, 6}, {1.75, 12.5}, {2, 15}, {2.25, 17.5}, {2.5, 19},
        {2.75, 21}, {3, 22}, {4, 22}, {4.5, 23}, {5, 25}, {10, 35}});
    public static final ShotTable kPassHood = kScoreHood;
    /**
     * Time of flight, seconds: from the shot until the ball comes down through the height of the
     * hub top. From the same sim model as kScoreSpeeds. Shoot on the move aims ahead by the
     * robot's velocity x this. Measure on the robot (video at 240 fps from the side).
     */
    public static final ShotTable kScoreTimeOfFlight = new ShotTable(new double[][] {
        {1.25, 0.925}, {1.5, 0.785}, {1.75, 0.69}, {2.0, 0.705}, {2.25, 0.715},
        {2.5, 0.74}, {2.75, 0.755}, {3.0, 0.785}, {3.25, 0.835}, {3.5, 0.88},
        {3.75, 0.925}, {4.0, 0.965}, {4.25, 0.995}, {4.5, 1.02}, {4.75, 1.035},
        {5.0, 1.045}, {5.25, 1.065}, {5.5, 1.09}, {5.75, 1.105}, {6.0, 1.125}});
    public static final ShotTable kPassTimeOfFlight = kScoreTimeOfFlight;
}
