package frc.robot.subsystems.drive;

import java.util.Optional;

import org.wpilib.math.geometry.Rotation2d;

/**
 * Keeps the robot pointed the same way while the driver drives without turning.
 *
 * <p>With the rotation stick at zero, nothing holds the heading: a bump, a wheel with less grip
 * or a small module error turns the robot a little, and it stays turned. Heading hold remembers
 * the heading and lets the heading controller keep it.
 *
 * <p>When to take the heading matters. If it is taken the moment the stick is released, the
 * robot is still turning, so it overshoots and then snaps back. So it is taken only when the turn
 * rate has dropped below a threshold. While the robot stands still there is no hold, so the
 * robot does not fight a push in the pit or at the start line.
 */
public class HeadingHold {
    private final double m_rateThreshold;
    private Rotation2d m_held = null;

    /** @param rateThreshold rad/s; the turn rate must fall below this before the heading is taken */
    public HeadingHold(double rateThreshold) {
        m_rateThreshold = rateThreshold;
    }

    /**
     * @param rotationCommand the driver's shaped rotation command (0 = no turn asked)
     * @param measuredRate the robot's measured turn rate, rad/s
     * @param heading the robot's heading now
     * @param translating true when the driver asks the robot to move
     * @return the heading to hold, or empty to let the driver's rotation command through
     */
    public Optional<Rotation2d> update(double rotationCommand, double measuredRate, Rotation2d heading, boolean translating) {
        if (rotationCommand != 0 || !translating) {
            m_held = null;
            return Optional.empty();
        }
        if (m_held == null && Math.abs(measuredRate) < m_rateThreshold) {
            m_held = heading;
        }
        return Optional.ofNullable(m_held);
    }

    /** Forget the held heading (for example after a pose reset or a snap command). */
    public void reset() {
        m_held = null;
    }
}
