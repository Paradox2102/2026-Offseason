package frc.robot.subsystems.intake;

/**
 * Intake values. Rack positions are meters of rack travel (0 = in); roller speeds are roller
 * rotations per second.
 */
public final class IntakeConstants {
    private IntakeConstants() {}

    // ---- Rack (Talons 21 and 22, Kraken X44, CAN_S4) ----

    /** Full stroke from the CAD, meters. The first extend measures the real one. */
    public static final double kRackStroke = 0.324;
    /**
     * Motor rotations per meter of rack travel. From the old code's full stroke (3.95 motor
     * rotations) over the CAD stroke. Check on the robot: push the rack out by hand and read the
     * motor rotations (Tuner X): rotations / 0.324 should be about this.
     */
    public static final double kRotationsPerMeter = 3.95 / kRackStroke;

    /** Start gains (volts). kV: X44 0.096 V per motor rps x kRotationsPerMeter. */
    public static final double kRackS = 0.25; // V
    public static final double kRackV = 1.2; // V per m/s
    public static final double kRackP = 30; // V per m of error
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
    /** A Motion Magic move is done within this distance of its goal, meters. */
    public static final double kAtGoalTolerance = 0.01;

    /** The sides may differ by this much (else an alert) ... meters ... */
    public static final double kSkewLimit = 0.025;
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
