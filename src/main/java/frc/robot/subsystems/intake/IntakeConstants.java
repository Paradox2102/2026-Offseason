package frc.robot.subsystems.intake;

import org.wpilib.math.system.DCMotor;

/**
 * Intake values. Rack positions are meters of rack travel (0 = in); roller speeds are roller
 * rotations per second.
 */
public final class IntakeConstants {
    private IntakeConstants() {}

    // ---- Rack (Talons 21 and 22, Kraken X44, CAN_S4) ----

    /** Full stroke from the CAD, meters. */
    public static final double kRackStroke = 0.324;
    /**
     * Out stop of each side, meters: where each side reads when it rests at its out stop. The
     * code trusts a stall short of these as "not out" (something may hold the side), so they
     * must be measured on the robot, with nothing in the way, not learned in a match (a ball on
     * the first extend would make a short stop look like the out stop). CAD values until then.
     *
     * <p>To measure: power the robot on with the intake fully in, enable, extend, and read
     * Intake/Rack/Left/Last Stall m and Intake/Rack/Right/Last Stall m (the reading at the stall,
     * before any fix). Do it three times (retract between) and write the largest of each here.
     */
    public static final double kOutStopLeft = kRackStroke;
    public static final double kOutStopRight = kRackStroke;
    /**
     * Motor rotations per meter of rack travel. From the old code's full stroke (3.95 motor
     * rotations) over the CAD stroke. Check on the robot: push the rack out by hand and read the
     * motor rotations (Tuner X): rotations / 0.324 should be about this.
     */
    public static final double kRotationsPerMeter = 3.95 / kRackStroke;

    /**
     * The rack slides on a 20 degree slope, out = downhill (CAD 2026-10-22: the rack plates move
     * 304.3 mm forward and 110.8 mm down between in and out). So gravity pushes the rack out.
     */
    public static final double kRackSlopeDeg = 20.0;
    /** Mass per side, kg (the old code's 5.4 kg rack, halved). Weigh the rack to check. */
    public static final double kRackSideMass = 2.7;
    /** Pinion pitch radius that gives kRotationsPerMeter with the 2.8:1 gears, meters. */
    public static final double kRackGearRatio = 2.8;
    public static final double kRackPinionRadius = kRackGearRatio / (2 * Math.PI * kRotationsPerMeter);
    private static final DCMotor kX44 = DCMotor.getKrakenX44Foc(1);
    /**
     * Motor current that balances gravity along the slope, per side, amps (about 8 A). Every
     * torque request adds -kGravityAmps, so the push and rest currents below are the NET push
     * into the stop. Without it, a 2 A "rest in" could not hold the rack against gravity.
     */
    public static final double kGravityAmps = kRackSideMass * 9.81 * Math.sin(Math.toRadians(kRackSlopeDeg))
        * kRackPinionRadius / (kRackGearRatio * kX44.Kt);
    /** The same as a voltage, for the Motion Magic gravity feedforward (kG), volts. */
    public static final double kGravityVolts = kGravityAmps * kX44.R;

    /** Start gains (volts). kV: X44 0.096 V per motor rps x kRotationsPerMeter. */
    public static final double kRackS = 0.25; // V
    public static final double kRackV = 1.2; // V per m/s
    public static final double kRackP = 30; // V per m of error
    /**
     * Acceleration feedforward, V per m/s^2: the side's mass on the pinion, through the gears, as
     * motor current, as volts. Without it the rack lags a goal that keeps moving (the pulse).
     */
    public static final double kRackA = kRackSideMass * kRackPinionRadius / (kRackGearRatio * kX44.Kt) * kX44.R;
    public static final double kRackCruise = 1.0; // m/s
    public static final double kRackAcceleration = 6.0; // m/s^2
    public static final double kRackStatorLimit = 40; // A per side
    public static final double kRackSupplyLimit = 30; // A per side

    /**
     * Push into a hard stop with this much current, amps. It must be clearly more than the
     * rack's friction (kS 0.25 V over the X44's ~0.043 ohm is ~6 A), or a push stalls before the
     * stop. Check on the robot: from 3.5 cm away, the push must reach the stop (Intake tab).
     */
    public static final double kPushCurrent = 12;
    /**
     * At rest the rack leans on its stop with this much current, amps. Low on purpose: a hit can
     * push the intake in without the motors fighting it (and without wasting battery).
     */
    public static final double kRestCurrent = 2;
    /** Pushed away from its stop, a side waits until it has been still this long, seconds, then goes back. */
    public static final double kSettleTime = 0.3;
    /** Stopped = slower than this, m/s ... */
    public static final double kStallSpeed = 0.005;
    /** ... for this long, seconds ... */
    public static final double kStallTime = 0.15;
    /** ... ignoring the first part of the push, seconds. */
    public static final double kStallIgnore = 0.2;
    /** A push that finds no stop in this time gives up, seconds. */
    public static final double kPushTimeout = 1.5;
    /** After a blocked or timed-out push, wait this long before trying again, seconds. */
    public static final double kRetryDelay = 2.0;
    /** Reboot flags in the first seconds after code start are the devices' own boot, seconds. */
    public static final double kStartupGrace = 3.0;
    /** A rack Talon that does not answer for this long counts as dead, seconds. */
    public static final double kDeadTime = 0.5;
    /** A Motion Magic move is done within this distance of its goal, meters. */
    public static final double kAtGoalTolerance = 0.01;

    /** The sides may differ by this much (else an alert) ... meters ... */
    public static final double kSkewLimit = 0.025;
    /**
     * "Fully out" for the trench and for shooting: each side within this of its out stop, meters.
     * The trench leaves 6 mm above the robot with the intake out, and each 1 cm the intake is in
     * raises it 3.4 mm (20 degree slope): 1.5 cm is 5 mm.
     */
    public static final double kExtendedTolerance = 0.015;
    /** ... for this long, seconds. */
    public static final double kSkewTime = 0.1;

    // ---- Rollers (Talon 20, Kraken X60, CAN_S4) ----

    /** Motor rotations per roller rotation: 24T to 36T. */
    public static final double kRollerRatio = 1.5;
    public static final double kRollerInSpeed = 26.7; // roller rps (was 40 motor rps)
    public static final double kRollerOutSpeed = -20; // roller rps (was -30 motor rps)
    public static final double kRollerS = 0.0; // V
    public static final double kRollerV = 0.225; // V per roller rps (was 0.15 per motor rps)
    public static final double kRollerP = 0.3; // V per roller rps of error
    public static final double kRollerAcceleration = 100; // roller rps per second
    public static final double kRollerSupplyLimit = 30; // A
    /** Moment of inertia of the rollers, for the sim only (the old code's value). */
    public static final double kRollerMoi = 0.093084;

    /** A jam: slower than this fraction of the target ... */
    public static final double kJamSpeedFraction = 0.2;
    /** ... with more stator current than this, amps ... */
    public static final double kJamCurrent = 50;
    /** ... for this long, seconds. Then run out for kUnjamTime. */
    public static final double kJamTime = 0.25;
    public static final double kUnjamTime = 0.3;
    /** Alert after this many jams within kJamWindow seconds. */
    public static final int kJamAlertCount = 3;
    public static final double kJamWindow = 5.0;
}
