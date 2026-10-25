package frc.robot.subsystems.superstructure;

/**
 * No aim without a field position. No hardware: unit tested.
 *
 * <p>The gyro gives the heading (driving needs only that), but an aim at a field point also needs
 * where the robot is. Until vision has given a pose, or an auto has set its start pose, the pose
 * is the field corner (0, 0). On the robot (2026-10-25, vision off, red alliance) the shot from
 * there was a pass to the red end, and the turret turned almost 180 degrees away from the hub.
 * So with no field position the turret holds still where it is and the hood stays down (the
 * shot is also blocked; see {@link Superstructure}).
 */
public final class AimGate {
    private double m_holdRot = Double.NaN;

    /**
     * @param poseKnown the pose has come from vision or an auto start
     * @param shotRot the shot's turret goal, rotations
     * @param turretNow where the turret is now, rotations
     * @return the turret goal, rotations
     */
    public double turretGoal(boolean poseKnown, double shotRot, double turretNow) {
        if (poseKnown) {
            m_holdRot = Double.NaN;
            return shotRot;
        }
        if (Double.isNaN(m_holdRot)) {
            m_holdRot = turretNow;
        }
        return m_holdRot;
    }

    /** The hood goal, degrees: down with no field position. */
    public static double hoodGoal(boolean poseKnown, double shotGoalDeg) {
        return poseKnown ? shotGoalDeg : 0;
    }
}
