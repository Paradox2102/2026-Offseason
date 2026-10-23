package frc.robot.subsystems.superstructure;

/**
 * When the intake packs and pulses during a shot. No hardware: unit tested.
 *
 * <p>The hopper empties from the feed end, and the last balls can sit where nothing pushes them.
 * So a while after the feed starts, the intake moves in a little (to the pack position), which
 * pushes the balls toward the feed, and then moves in and out around that position (the pulse),
 * which shakes loose a ball that is stuck. Not so far in that it blocks the shots.
 *
 * <ul>
 *   <li>Before {@code delay} after the first feed: no pack (the intake stays fully out).
 *   <li>Then the pack position, for {@code settleTime} (the rack gets there).
 *   <li>Then the pack position plus a sine of the pulse amplitude, starting at 0.
 * </ul>
 */
public class PackPulse {
    private final double m_delay;
    private final double m_settleTime;

    /**
     * @param delay the pack starts this long after the first feed of the shot, seconds
     * @param settleTime the pulse starts this long after the pack starts, seconds
     */
    public PackPulse(double delay, double settleTime) {
        m_delay = delay;
        m_settleTime = settleTime;
    }

    /**
     * @param sinceFeedStart seconds since the first feed of this shot (NaN: no feed yet)
     * @param amplitude pulse amplitude, meters
     * @param rateHz pulses per second
     * @return the offset from the pack position, meters (positive = out), or NaN for no pack
     */
    public double offset(double sinceFeedStart, double amplitude, double rateHz) {
        if (Double.isNaN(sinceFeedStart) || sinceFeedStart < m_delay) {
            return Double.NaN;
        }
        double pulseTime = sinceFeedStart - m_delay - m_settleTime;
        if (pulseTime < 0) {
            return 0;
        }
        return amplitude * Math.sin(2 * Math.PI * rateHz * pulseTime);
    }
}
