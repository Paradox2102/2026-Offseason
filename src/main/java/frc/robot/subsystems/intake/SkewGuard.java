package frc.robot.subsystems.intake;

/**
 * Watches that the two sides of the intake rack stay together.
 *
 * <p>The two sides have no shaft between them: each has its own motor and its own position loop.
 * If one side binds or loses power, the other keeps going and the rack twists. When the sides
 * differ by more than the limit for longer than the time, this trips and stays tripped until
 * {@link #reset}, so the rack stops instead of twisting further.
 */
public class SkewGuard {
    private final double m_limit;
    private final double m_time;
    private double m_overSince = Double.NaN;
    private boolean m_tripped = false;

    /**
     * @param limit largest allowed difference between the sides, meters
     * @param time how long the difference may last, seconds
     */
    public SkewGuard(double limit, double time) {
        m_limit = limit;
        m_time = time;
    }

    /** @return true when tripped */
    public boolean update(double now, double left, double right) {
        if (m_tripped) {
            return true;
        }
        if (Math.abs(left - right) <= m_limit) {
            m_overSince = Double.NaN;
            return false;
        }
        if (Double.isNaN(m_overSince)) {
            m_overSince = now;
        }
        m_tripped = now - m_overSince >= m_time;
        return m_tripped;
    }

    public void reset() {
        m_tripped = false;
        m_overSince = Double.NaN;
    }
}
