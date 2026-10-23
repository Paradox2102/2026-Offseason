package frc.robot.calibration;

/**
 * Counts the balls a shot fires from the flywheel's speed: each ball takes some speed from the
 * wheel. No hardware: unit tested.
 *
 * <p>A dip starts when the speed falls under {@code dropFraction} of the goal, and the next one can
 * start only after the speed is back over {@code recoverFraction}. The spin-up is not a dip: the
 * wheel must have been at speed first. The fire rate is the dips per
 * second from the first dip to the last.
 */
public class DipCounter {
    private final double m_dropFraction;
    private final double m_recoverFraction;
    /** True until the wheel is at speed, and in a dip. */
    private boolean m_inDip = true;
    private int m_count = 0;
    private double m_first = Double.NaN;
    private double m_last = Double.NaN;

    public DipCounter(double dropFraction, double recoverFraction) {
        m_dropFraction = dropFraction;
        m_recoverFraction = recoverFraction;
    }

    /** Start again (a new test). */
    public void reset() {
        m_inDip = true;
        m_count = 0;
        m_first = Double.NaN;
        m_last = Double.NaN;
    }

    /**
     * @param speed the flywheel speed now
     * @param goal the flywheel goal (0 or less: not spinning, nothing counts)
     */
    public void update(double now, double speed, double goal) {
        if (goal <= 0) {
            m_inDip = true; // spinning up again: wait until it is at speed
            return;
        }
        if (!m_inDip && speed < m_dropFraction * goal) {
            m_inDip = true;
            m_count++;
            if (Double.isNaN(m_first)) {
                m_first = now;
            }
            m_last = now;
        } else if (m_inDip && speed > m_recoverFraction * goal) {
            m_inDip = false;
        }
    }

    public int count() {
        return m_count;
    }

    /** Balls per second from the first dip to the last (NaN with fewer than 2 dips). */
    public double rate() {
        return m_count < 2 || m_last <= m_first ? Double.NaN : (m_count - 1) / (m_last - m_first);
    }
}
