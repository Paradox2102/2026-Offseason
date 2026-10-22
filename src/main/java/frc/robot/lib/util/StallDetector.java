package frc.robot.lib.util;

/**
 * Finds the moment a mechanism stops moving: "speed under a limit for a hold time".
 *
 * <p>Used to find a hard stop (push gently until the mechanism stops) and to find a jam (a roller
 * that should spin but does not). The first part of each push is ignored, because every motor
 * starts at speed 0.
 */
public class StallDetector {
    private final double m_speedLimit;
    private final double m_holdTime;
    private final double m_ignoreTime;
    private double m_startTime = 0;
    private double m_slowSince = Double.NaN;

    /**
     * @param speedLimit slower than this (absolute) counts as stopped
     * @param holdTime it must stay stopped this long, seconds
     * @param ignoreTime ignore this long after {@link #reset}, seconds
     */
    public StallDetector(double speedLimit, double holdTime, double ignoreTime) {
        m_speedLimit = speedLimit;
        m_holdTime = holdTime;
        m_ignoreTime = ignoreTime;
    }

    /** Start watching a new push. */
    public void reset(double now) {
        m_startTime = now;
        m_slowSince = Double.NaN;
    }

    /** @return true once the speed has stayed under the limit for the hold time */
    public boolean update(double now, double speed) {
        if (now - m_startTime < m_ignoreTime || Math.abs(speed) >= m_speedLimit) {
            m_slowSince = Double.NaN;
            return false;
        }
        if (Double.isNaN(m_slowSince)) {
            m_slowSince = now;
        }
        return now - m_slowSince >= m_holdTime;
    }
}
