package frc.robot.subsystems.shooter;

import java.util.ArrayDeque;
import java.util.Arrays;

/**
 * What each stage of the feed path does, loop by loop. No hardware: unit tested.
 *
 * <ul>
 *   <li><b>Staged stop.</b> When the feed stops, every stage brakes except the last one (the one
 *       that feeds the flywheel), which runs on for a short time. A ball already at the last stage
 *       then goes on into the wheel instead of being held against it (it would be scrubbed).
 *   <li><b>Jams.</b> While feeding, a stage that stalls (the Feeder decides: slow with a high
 *       current) for the jam time is a jam: every stage runs back for the unjam time, then feeds
 *       again. After an unjam the jam check waits its full time again. Several jams in a short
 *       window raise an alert: then a short reverse does not fix it.
 *   <li><b>Eject.</b> Every stage runs back; no jam check, and no run-on after it.
 * </ul>
 */
public class FeedLogic {
    public enum Mode { FEED, STOP, EJECT }

    public enum Out { FEED, REVERSE, BRAKE }

    private final int m_stages;
    private final double m_runOn;
    private final double m_jamTime;
    private final double m_unjamTime;
    private final int m_alertCount;
    private final double m_window;
    private final double[] m_stalledSince;
    private final ArrayDeque<Double> m_jamTimes = new ArrayDeque<>();
    private double m_runOnUntil = Double.NEGATIVE_INFINITY;
    private double m_unjamUntil = Double.NEGATIVE_INFINITY;
    private int m_jammedStage = -1;

    /**
     * @param stages number of stages; the last one feeds the flywheel
     * @param runOn the last stage runs on this long after a feed stops, seconds
     * @param jamTime a stall this long is a jam, seconds
     * @param unjamTime every stage runs back this long after a jam, seconds
     * @param alertCount this many jams within {@code window} seconds raise the alert
     */
    public FeedLogic(int stages, double runOn, double jamTime, double unjamTime, int alertCount, double window) {
        m_stages = stages;
        m_runOn = runOn;
        m_jamTime = jamTime;
        m_unjamTime = unjamTime;
        m_alertCount = alertCount;
        m_window = window;
        m_stalledSince = new double[stages];
        resetStalls();
    }

    /**
     * @param stalled per stage: slow with a high current now
     * @return per stage, what to do now
     */
    public Out[] update(double now, Mode mode, boolean[] stalled) {
        switch (mode) {
            case EJECT -> {
                resetStalls();
                m_runOnUntil = Double.NEGATIVE_INFINITY;
                m_unjamUntil = Double.NEGATIVE_INFINITY;
                return all(Out.REVERSE);
            }
            case STOP -> {
                resetStalls();
                if (now < m_unjamUntil) {
                    // Stopped in the middle of an unjam: the ball is going back, do not push it on.
                    m_unjamUntil = Double.NEGATIVE_INFINITY;
                    m_runOnUntil = Double.NEGATIVE_INFINITY;
                }
                var out = all(Out.BRAKE);
                if (now < m_runOnUntil) {
                    out[m_stages - 1] = Out.FEED;
                }
                return out;
            }
            default -> {
                if (now < m_unjamUntil) {
                    resetStalls(); // after the unjam the check waits its full time again
                    return all(Out.REVERSE);
                }
                for (int i = 0; i < m_stages; i++) {
                    if (!stalled[i]) {
                        m_stalledSince[i] = Double.NaN;
                        continue;
                    }
                    if (Double.isNaN(m_stalledSince[i])) {
                        m_stalledSince[i] = now;
                    }
                    if (now - m_stalledSince[i] >= m_jamTime) {
                        m_jammedStage = i;
                        m_jamTimes.addLast(now);
                        m_unjamUntil = now + m_unjamTime;
                        m_runOnUntil = Double.NEGATIVE_INFINITY;
                        resetStalls();
                        return all(Out.REVERSE);
                    }
                }
                m_runOnUntil = now + m_runOn;
                return all(Out.FEED);
            }
        }
    }

    /** The stage of the last jam (-1: none yet). */
    public int jammedStage() {
        return m_jammedStage;
    }

    /** Jams within the window before {@code now}. */
    public int recentJams(double now) {
        while (!m_jamTimes.isEmpty() && now - m_jamTimes.peekFirst() > m_window) {
            m_jamTimes.removeFirst();
        }
        return m_jamTimes.size();
    }

    public boolean jamAlert(double now) {
        return recentJams(now) >= m_alertCount;
    }

    public boolean isUnjamming(double now) {
        return now < m_unjamUntil;
    }

    private void resetStalls() {
        Arrays.fill(m_stalledSince, Double.NaN);
    }

    private Out[] all(Out out) {
        var result = new Out[m_stages];
        Arrays.fill(result, out);
        return result;
    }
}
