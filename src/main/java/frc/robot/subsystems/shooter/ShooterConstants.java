package frc.robot.subsystems.shooter;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

/**
 * Shooter values. Turret angles are turret rotations (0 = the jig position, pointing to the
 * robot's left); hood angles are degrees above its lower stop; flywheel speeds are wheel rps;
 * feed speeds are motor rps (the old code's units).
 */
public final class ShooterConstants {
    private ShooterConstants() {}

    // ---- Turret (Talon 44, Kraken X60, CAN_S3) ----

    /** Motor rotations per turret rotation: 12:60, 14:28, 14:95 (CAD; same as the old code). */
    public static final double kTurretRatio = 67.86;
    /** In the jig at power-on, the shooter points to the robot's left: turret 0 = robot +90 degrees. */
    public static final double kTurretZeroOnRobotDeg = 90;
    /** Cable chain limits from the jig position (old code: +-250 degrees). Check on the robot. */
    public static final double kTurretMinRot = -250.0 / 360;
    public static final double kTurretMaxRot = 250.0 / 360;
    /** Turret pivot from the robot center, meters (old code; CAD turret gear center agrees within 2 mm). */
    public static final Translation2d kTurretOffset =
        new Translation2d(Units.inchesToMeters(-5.965), Units.inchesToMeters(7.5429));
    /** Aim this far ahead of the robot's turn, seconds (the turret lags behind a turning robot). */
    public static final double kTurretLeadTime = 0.1;
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
    /** Push down into the lower stop with this current to find zero, amps. */
    public static final double kHoodHomeCurrent = -3;
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

    // ---- Feed path, motor rps (old code values) ----

    public static final double kIndexerRollerRps = 25; // Talon 30, CAN_S1
    /**
     * Talons 31 and 32 (X44, CAN_S1). The old code asked for 20 with a duty-cycle kV of 0.1 per
     * rps (= 200 % output), so the belts really ran flat out. 90 motor rps keeps that speed with
     * a real velocity loop (X44 free speed ~125 rps).
     */
    public static final double kBeltTunnelRps = 90;
    public static final double kScottyRps = 30; // Talon 45 (code name "goUp")
    public static final double kVerticalRollerRps = 30; // Talon 40 (code name "kicker")
    /** Eject: every feed motor runs out at this fraction of its feed speed. */
    public static final double kEjectFraction = -0.5;

    // ---- Shot tables: {distance m, value} ----

    /** Old code's shot speeds in MOTOR rps; converted to wheel rps below. Tune on the robot. */
    private static final double[][] kScoreSpeedsMotorRps = {
        {1.125, 40}, {2.25, 45}, {2.75, 47}, {3.25, 50}, {4, 55}, {4.75, 60}};
    public static final ShotTable kScoreSpeeds = new ShotTable(kScoreSpeedsMotorRps).scaled(1 / kFlywheelRatio);
    public static final ShotTable kPassSpeeds = kScoreSpeeds;
    /** Old code's hood angles, degrees. */
    public static final ShotTable kScoreHood = new ShotTable(new double[][] {
        {1.125, 0}, {1.25, 0}, {1.5, 10}, {1.75, 12.5}, {2, 15}, {2.25, 17.5}, {2.5, 19},
        {2.75, 21}, {3, 22}, {4, 22}, {4.5, 23}, {5, 25}, {10, 35}});
    public static final ShotTable kPassHood = kScoreHood;
    /** Time of flight, seconds. 0 = not measured = no shoot-on-the-move lead. */
    public static final ShotTable kScoreTimeOfFlight = new ShotTable(new double[][] {{0, 0}});
    public static final ShotTable kPassTimeOfFlight = kScoreTimeOfFlight;
}
