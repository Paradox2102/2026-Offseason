package frc.robot.subsystems.vision;

import org.wpilib.math.geometry.Rotation2d;

/**
 * The heading to judge single-tag frames against.
 *
 * <p>A single tag gives two possible poses, and the one nearer the robot heading is used (see
 * {@link VisionCamera}). Normally that heading is the estimator's. But the estimator heading can
 * be far off: after a Pigeon reboot (its yaw restarts at about -89 degrees), or before the first
 * pose reset (the heading is the boot value). Then every single tag would be rejected, or the
 * wrong solution picked. Multi-tag frames measure the heading on their own, so when one says the
 * estimator is more than {@link VisionFilter#kMaxSingleTagHeadingError} off, single tags are judged
 * against the estimator heading plus that offset for a short time.
 *
 * <p>Smaller offsets are ignored: they are more likely camera angles a few degrees off than a
 * wrong gyro, and the gyro is better at small errors.
 *
 * <p>This class has no hardware in it, so the unit tests can check it.
 */
public final class HeadingReference {
    /** An offset is used for this long after the multi-tag frame that measured it, seconds. */
    public static final double kHoldSeconds = 1.0;

    private double m_offsetRad = 0;
    private double m_observedAt = Double.NEGATIVE_INFINITY;

    /**
     * A multi-tag frame captured at {@code time} gave {@code visionYaw}; the estimator heading at
     * that time was {@code estimatorYaw}.
     */
    public void observe(double time, Rotation2d visionYaw, Rotation2d estimatorYaw) {
        m_offsetRad = visionYaw.minus(estimatorYaw).getRadians();
        m_observedAt = time;
    }

    /** The heading at {@code time} to judge a single tag against, from the estimator's. */
    public Rotation2d correct(double time, Rotation2d estimatorYaw) {
        boolean recent = time - m_observedAt <= kHoldSeconds;
        if (recent && Math.abs(m_offsetRad) > VisionFilter.kMaxSingleTagHeadingError) {
            return estimatorYaw.plus(Rotation2d.fromRadians(m_offsetRad));
        }
        return estimatorYaw;
    }
}
