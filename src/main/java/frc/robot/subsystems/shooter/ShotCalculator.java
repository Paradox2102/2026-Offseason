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
 *   <li><b>Shoot on the move:</b> a ball keeps the robot's speed when it leaves. So aim at the
 *       target moved back by (robot field velocity x time of flight). The time of flight depends
 *       on the distance, which depends on the aim point, so this is repeated a few times.
 *   <li><b>Turret angle:</b> from the turret pivot (not the robot center) to the aim point, minus
 *       the robot heading (a little ahead, for a turning robot), minus the jig angle.
 *   <li><b>Flywheel and hood:</b> from the distance, in measured tables.
 * </ol>
 */
public class ShotCalculator {
    /** Everything about one shot. Turret in rotations from the jig; flywheel in wheel rps. */
    /**
     * @param turretRot the turret goal, with the lead for a turning robot (for the command)
     * @param turretAimRot the angle that points at the aim point now, no lead (for "ready")
     */
    public record Shot(Translation2d target, boolean scoring, double distance, double turretRot,
                       double turretAimRot, double flywheelRps, double hoodDeg) {}

    private static final int kLeadIterations = 3;

    private final ShotTable m_scoreSpeeds;
    private final ShotTable m_scoreHood;
    private final ShotTable m_scoreTimeOfFlight;
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

    /** True while the robot is in our alliance zone (where shots go into our hub). */
    public static boolean inOurZone(Pose2d robot, boolean red) {
        return red ? robot.getX() > FieldConstants.k_redZoneTransition : robot.getX() < FieldConstants.k_blueZoneTransition;
    }

    /**
     * @param robot robot pose on the field
     * @param fieldVelocity robot velocity in the field frame
     * @param omega robot turn rate, rad/s
     * @param red our alliance is red (false also when not known)
     * @param currentTurretRot where the turret is now, rotations (to pick which way round)
     */
    public Shot calculate(Pose2d robot, ChassisVelocities fieldVelocity, double omega, boolean red, double currentTurretRot) {
        boolean scoring = inOurZone(robot, red);
        Translation2d target;
        if (scoring) {
            target = FieldConstants.hub(red);
        } else if (robot.getY() > FieldConstants.k_yCenterline) {
            target = red ? FieldConstants.k_redDepot : FieldConstants.k_blueDepot;
        } else {
            target = red ? FieldConstants.k_redOutpost : FieldConstants.k_blueOutpost;
        }
        Aim aim = aim(robot, target, fieldVelocity, omega, scoring ? m_scoreTimeOfFlight : m_passTimeOfFlight, currentTurretRot);
        double distance = aim.distance();
        double flywheel = (scoring ? m_scoreSpeeds : m_passSpeeds).get(distance);
        double hood = Math.min((scoring ? m_scoreHood : m_passHood).get(distance), ShooterConstants.kHoodMaxDeg);
        return new Shot(target, scoring, distance, aim.turretRot(), aim.turretAimRot(), flywheel, hood);
    }

    /**
     * Turret angles (rotations from the jig) and distance to the aim point, meters.
     *
     * @param turretRot goal with the turn lead: the turret lags a turning robot, so it is asked
     *     to be a little ahead
     * @param turretAimRot the angle that points at the aim point now: a shot is ready only when
     *     the turret is here
     */
    public record Aim(double turretRot, double turretAimRot, double distance) {}

    /** Aim the turret at a target, with the lead for a moving and turning robot. */
    public static Aim aim(Pose2d robot, Translation2d target, ChassisVelocities fieldVelocity, double omega,
                          ShotTable timeOfFlight, double currentTurretRot) {
        Translation2d pivot = robot.getTranslation().plus(ShooterConstants.kTurretOffset.rotateBy(robot.getRotation()));
        Translation2d drift = new Translation2d(fieldVelocity.vx, fieldVelocity.vy);
        Translation2d aim = target;
        for (int i = 0; i < kLeadIterations; i++) {
            double tof = timeOfFlight.get(aim.getDistance(pivot));
            aim = target.minus(drift.times(tof));
        }

        double fieldAngle = aim.minus(pivot).getAngle().map(a -> a.getRadians()).orElse(robot.getRotation().getRadians());
        double heading = robot.getRotation().getRadians();
        double lead = omega * ShooterConstants.kTurretLeadTime;
        double aimRad = fieldAngle - heading - Math.toRadians(ShooterConstants.kTurretZeroOnRobotDeg);
        double turretAimRot = TurretMath.chooseAngle(aimRad / (2 * Math.PI), currentTurretRot,
            ShooterConstants.kTurretAimMinRot, ShooterConstants.kTurretAimMaxRot);
        // The goal is the aim angle less the lead (the robot turns +omega, so the turret must turn
        // -omega relative to it), kept inside the limits.
        double turretRot = Math.clamp(turretAimRot - lead / (2 * Math.PI),
            ShooterConstants.kTurretAimMinRot, ShooterConstants.kTurretAimMaxRot);
        return new Aim(turretRot, turretAimRot, aim.getDistance(pivot));
    }
}
