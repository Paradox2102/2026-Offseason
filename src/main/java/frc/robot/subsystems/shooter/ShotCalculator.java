package frc.robot.subsystems.shooter;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import frc.robot.constants.FieldConstants;

/**
 * Where to shoot, and how: the target, the turret angle, the flywheel speed and the hood angle.
 * No hardware: unit tested.
 *
 * <ol>
 *   <li><b>Target:</b> our hub while the robot is in our alliance zone; otherwise a pass target,
 *       the depot or the outpost by the robot's y.
 *   <li><b>Shoot on the move:</b> a ball keeps the velocity of the point it leaves from: the
 *       turret axis, which moves with the robot and, as it is off the robot center, with its
 *       turn. So aim at the target moved back by (that velocity x time of flight). The time of
 *       flight depends on the distance, which depends on the aim point, so this is repeated until
 *       it settles. Everything is worked out from where the robot will be when the ball leaves
 *       ({@link ShooterConstants#kShotLatency} ahead).
 *   <li><b>Turret angle:</b> from the turret axis (not the robot center) to the aim point, minus
 *       the robot heading, minus the jig angle. The goal leads that angle by its rate of change
 *       (the robot's turn, and the aim point's bearing changing as the robot drives past) x the
 *       turret's lag ({@link ShooterConstants#kTurretLeadTime}).
 *   <li><b>Flywheel and hood:</b> from the distance, in measured tables.
 * </ol>
 */
public class ShotCalculator {
    /** Everything about one shot. Turret in rotations from the jig; flywheel in wheel rps. */
    /**
     * @param turretRot the turret goal, with the lead for a turning robot (for the command)
     * @param turretAimRot the angle that points at the aim point now, no lead (for "ready")
     */
    /**
     * @param flywheelRps the speed this shot needs now (for "ready")
     * @param hoodDeg the hood angle this shot needs now (for "ready")
     * @param flywheelGoalRps the flywheel goal: for the distance kFlywheelLeadTime ahead
     * @param hoodGoalDeg the hood goal: for the distance kHoodLeadTime ahead
     * @param distanceRate how fast the distance changes, m/s (negative = closing)
     */
    public record Shot(Translation2d target, boolean scoring, double distance, double turretRot,
                       double turretAimRot, double flywheelRps, double hoodDeg, Translation2d aimPoint,
                       double flywheelGoalRps, double hoodGoalDeg, double distanceRate) {}

    /** The lead is solved again until it moves less than this, meters (or for kMaxLeadIterations). */
    private static final double kLeadTolerance = 0.001;
    private static final int kMaxLeadIterations = 8;

    private boolean m_shootOnTheMove = true;

    private ShotTable m_scoreSpeeds;
    private ShotTable m_scoreHood;
    private ShotTable m_scoreTimeOfFlight;
    private final ShotTable m_passSpeeds;
    private final ShotTable m_passHood;
    private final ShotTable m_passTimeOfFlight;

    public ShotCalculator() {
        this(ShooterConstants.kScoreSpeeds, ShooterConstants.kScoreHood, ShooterConstants.kScoreTimeOfFlight,
            ShooterConstants.kPassSpeeds, ShooterConstants.kPassHood, ShooterConstants.kPassTimeOfFlight);
    }

    public ShotCalculator(ShotTable scoreSpeeds, ShotTable scoreHood, ShotTable scoreTimeOfFlight,
                          ShotTable passSpeeds, ShotTable passHood, ShotTable passTimeOfFlight) {
        m_scoreSpeeds = scoreSpeeds;
        m_scoreHood = scoreHood;
        m_scoreTimeOfFlight = scoreTimeOfFlight;
        m_passSpeeds = passSpeeds;
        m_passHood = passHood;
        m_passTimeOfFlight = passTimeOfFlight;
    }

    /** New score tables (points measured on the robot), used from the next shot on. */
    public void setScoreTables(ShotTable speeds, ShotTable hood, ShotTable timeOfFlight) {
        m_scoreSpeeds = speeds;
        m_scoreHood = hood;
        m_scoreTimeOfFlight = timeOfFlight;
    }

    /** A still score shot at this distance: {flywheel wheel rps, hood degrees}. */
    public double[] scoreAt(double distance) {
        return new double[] {m_scoreSpeeds.get(distance), Math.min(m_scoreHood.get(distance), ShooterConstants.kHoodMaxDeg)};
    }

    /**
     * Off: aim as if the robot were still (no lead for its velocity or the latency; the turn lead
     * stays). For a robot whose lead is wrong, until it is measured.
     */
    public void setShootOnTheMove(boolean on) {
        m_shootOnTheMove = on;
    }

    public boolean isShootOnTheMove() {
        return m_shootOnTheMove;
    }

    /** True while the robot is in our alliance zone (where shots go into our hub). */
    public static boolean inOurZone(Pose2d robot, boolean red) {
        return red ? robot.getX() > FieldConstants.k_redZoneTransition : robot.getX() < FieldConstants.k_blueZoneTransition;
    }

    /**
     * True if the robot is in our zone now, or will be within {@code lookahead} seconds at its
     * present velocity: on the way home the shot is set up for the hub before the robot gets there.
     */
    public static boolean inOurZoneSoon(Pose2d robot, ChassisVelocities fieldVelocity, boolean red, double lookahead) {
        var ahead = new Pose2d(robot.getX() + fieldVelocity.vx * lookahead, robot.getY() + fieldVelocity.vy * lookahead,
            robot.getRotation());
        return inOurZone(robot, red) || inOurZone(ahead, red);
    }

    /** The shot for where the robot is now (no look ahead). */
    public Shot calculate(Pose2d robot, ChassisVelocities fieldVelocity, double omega, boolean red, double currentTurretRot) {
        return calculate(robot, fieldVelocity, omega, red, currentTurretRot, 0);
    }

    /**
     * @param robot robot pose on the field
     * @param fieldVelocity robot velocity in the field frame
     * @param omega robot turn rate, rad/s
     * @param red our alliance is red (false also when not known)
     * @param currentTurretRot where the turret is now, rotations (to pick which way round)
     * @param zoneLookahead pick the hub if the robot will be in our zone this many seconds ahead
     *     (0 = only if it is in now). The aim itself is always from where the robot is now.
     */
    public Shot calculate(Pose2d robot, ChassisVelocities fieldVelocity, double omega, boolean red, double currentTurretRot,
                          double zoneLookahead) {
        boolean scoring = inOurZoneSoon(robot, fieldVelocity, red, zoneLookahead);
        Translation2d target;
        if (scoring) {
            target = FieldConstants.hub(red);
        } else if (robot.getY() > FieldConstants.k_yCenterline) {
            target = red ? FieldConstants.k_redDepot : FieldConstants.k_blueDepot;
        } else {
            target = red ? FieldConstants.k_redOutpost : FieldConstants.k_blueOutpost;
        }
        var leadVelocity = m_shootOnTheMove ? fieldVelocity : new ChassisVelocities();
        double latency = m_shootOnTheMove ? ShooterConstants.kShotLatency : 0;
        Aim aim = aim(robot, target, leadVelocity, omega, scoring ? m_scoreTimeOfFlight : m_passTimeOfFlight,
            currentTurretRot, latency);
        double distance = aim.distance();
        var speeds = scoring ? m_scoreSpeeds : m_passSpeeds;
        var hoods = scoring ? m_scoreHood : m_passHood;
        double flywheel = speeds.get(distance);
        double hood = Math.min(hoods.get(distance), ShooterConstants.kHoodMaxDeg);
        // The hood and the flywheel lag a distance that keeps changing: their goals are for the
        // distance a little ahead.
        double flywheelGoal = speeds.get(distance + aim.distanceRate() * ShooterConstants.kFlywheelLeadTime);
        double hoodGoal = Math.min(hoods.get(distance + aim.distanceRate() * ShooterConstants.kHoodLeadTime),
            ShooterConstants.kHoodMaxDeg);
        return new Shot(target, scoring, distance, aim.turretRot(), aim.turretAimRot(), flywheel, hood, aim.aimPoint(),
            flywheelGoal, hoodGoal, aim.distanceRate());
    }

    /** The turret is on target for a shot at this distance within this angle, degrees. */
    public static double turretToleranceDeg(double distance) {
        return Math.max(ShooterConstants.kTurretMinToleranceDeg,
            Math.toDegrees(Math.atan(ShooterConstants.kTurretToleranceMeters / Math.max(distance, 0.1))));
    }

    /**
     * Turret angles (rotations from the jig) and distance to the aim point, meters.
     *
     * @param turretRot goal with the lead: the turret lags a turning robot (and a target whose
     *     bearing changes), so it is asked to be a little ahead
     * @param turretAimRot the angle that points at the aim point: a shot is ready only when the
     *     turret is here
     * @param aimPoint the target moved back for the shooter's velocity (the target when still)
     * @param distanceRate how fast the distance to the aim point changes, m/s (negative = closing)
     */
    public record Aim(double turretRot, double turretAimRot, double distance, Translation2d aimPoint, double distanceRate) {}

    /** Aim the turret at a target, with the lead for a moving and turning robot and the shot latency. */
    public static Aim aim(Pose2d robot, Translation2d target, ChassisVelocities fieldVelocity, double omega,
                          ShotTable timeOfFlight, double currentTurretRot) {
        return aim(robot, target, fieldVelocity, omega, timeOfFlight, currentTurretRot, ShooterConstants.kShotLatency);
    }

    /**
     * @param latency seconds from now until the ball leaves: the shot is worked out from where
     *     the robot will be then
     */
    public static Aim aim(Pose2d robot, Translation2d target, ChassisVelocities fieldVelocity, double omega,
                          ShotTable timeOfFlight, double currentTurretRot, double latency) {
        Translation2d offset = ShooterConstants.kTurretOffset.rotateBy(robot.getRotation());
        // Velocity of the turret axis: the robot's, plus omega x (its offset from the center).
        double vx = fieldVelocity.vx - omega * offset.getY();
        double vy = fieldVelocity.vy + omega * offset.getX();
        // Where the turret axis is, and where the robot points, when the ball leaves.
        Translation2d pivot = robot.getTranslation().plus(offset).plus(new Translation2d(vx * latency, vy * latency));
        double heading = robot.getRotation().getRadians() + omega * latency;

        Translation2d aim = target;
        for (int i = 0; i < kMaxLeadIterations; i++) {
            double tof = timeOfFlight.get(aim.getDistance(pivot));
            double drift = tof * Math.max(0, 1 - ShooterConstants.kLeadDecay * tof);
            var next = new Translation2d(target.getX() - vx * drift, target.getY() - vy * drift);
            boolean settled = next.getDistance(aim) < kLeadTolerance;
            aim = next;
            if (settled) {
                break;
            }
        }

        Translation2d r = aim.minus(pivot);
        double fieldAngle = r.getAngle().map(a -> a.getRadians()).orElse(heading);
        double aimRad = fieldAngle - heading - Math.toRadians(ShooterConstants.kTurretZeroOnRobotDeg);
        double turretAimRot = TurretMath.chooseAngle(aimRad / (2 * Math.PI), currentTurretRot,
            ShooterConstants.kTurretAimMinRot, ShooterConstants.kTurretAimMaxRot);
        // How fast the turret angle must change: the aim point's bearing (it moves at -v relative
        // to the turret axis) less the robot's turn. The goal is that far ahead of the aim.
        double r2 = r.getX() * r.getX() + r.getY() * r.getY();
        double bearingRate = r2 < 1e-6 ? 0 : (r.getY() * vx - r.getX() * vy) / r2;
        double distanceRate = r2 < 1e-6 ? 0 : -(r.getX() * vx + r.getY() * vy) / Math.sqrt(r2);
        double lead = (bearingRate - omega) * ShooterConstants.kTurretLeadTime;
        double turretRot = Math.clamp(turretAimRot + lead / (2 * Math.PI),
            ShooterConstants.kTurretAimMinRot, ShooterConstants.kTurretAimMaxRot);
        return new Aim(turretRot, turretAimRot, Math.sqrt(r2), aim, distanceRate);
    }
}
