package frc.robot.subsystems.shooter;

/**
 * The hood pushes hard, short of its goal, and does not move: it is on a stop (most likely
 * kHoodRatio is wrong, or something blocks it). No hardware: unit tested.
 *
 * <p>It must last {@code holdTime} without a break, counted from {@link #reset} at the latest. (A
 * WPILib Debouncer counts from when it was made: on the robot, 2026-10-25, the first loop after
 * homing fired at once, on the homing push's own current.) Moving does not count: speeding up
 * toward a far goal also takes current.
 */
public class HoodStrain {
    private final double m_minAmps;
    private final double m_minErrorDeg;
    private final double m_maxDegPerSec;
    private final double m_holdTime;
    private double m_since = Double.NaN;

    /**
     * @param minAmps stator current above this ...
     * @param minErrorDeg ... more than this from the goal ...
     * @param maxDegPerSec ... slower than this ...
     * @param holdTime ... for this long, seconds
     */
    public HoodStrain(double minAmps, double minErrorDeg, double maxDegPerSec, double holdTime) {
        m_minAmps = minAmps;
        m_minErrorDeg = minErrorDeg;
        m_maxDegPerSec = maxDegPerSec;
        m_holdTime = holdTime;
    }

    /** Start again (the hood has just found its zero). */
    public void reset() {
        m_since = Double.NaN;
    }

    /** @return true while the strain has lasted the hold time */
    public boolean update(double now, double statorAmps, double errorDeg, double degPerSec) {
        boolean straining = Math.abs(statorAmps) > m_minAmps && Math.abs(errorDeg) > m_minErrorDeg
            && Math.abs(degPerSec) < m_maxDegPerSec;
        if (!straining) {
            m_since = Double.NaN;
            return false;
        }
        if (Double.isNaN(m_since)) {
            m_since = now;
        }
        return now - m_since >= m_holdTime;
    }
}
