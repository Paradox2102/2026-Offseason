package frc.robot.subsystems.intake;

/**
 * What one side of the intake rack knows about its own position. No hardware: unit tested.
 *
 * <p>The intake gets hit. A hit can push it in (the motor is backdriven: the reading follows) or
 * make a gear skip teeth on the rack (the reading is then wrong). One side can skip while the
 * other does not, so each side keeps its own knowledge, and trusts a reading only when the side
 * stalls against a hard stop:
 * <ul>
 *   <li>At boot the side reads 0, and 0 is "in" (the robot boots with the intake in).
 *   <li>Every move ends with a push into its stop. At the stall the reading is set to the stop:
 *       0 for the in stop, the measured stroke for the out stop. The first out stall measures the
 *       stroke.
 *   <li>A stall far from where the side expected it means skipped teeth (an alert), unless the
 *       side's Talon rebooted ("lost"): then a difference is expected.
 * </ul>
 */
public class RackSide {
    /** What to do after a push stopped at a hard stop. */
    public record StopResult(double setPosition, String alert) {}

    /** The push starts this far before the stop, meters. */
    public static final double kApproachDistance = 0.035;
    /** A stall further than this from the expected stop means skipped teeth, meters. */
    public static final double kSkipTolerance = 0.01;
    /** Further than this from its stop at rest, the side was pushed away and goes back, meters. */
    public static final double kPushedAway = 0.02;
    /** A first out stop outside these fractions of the CAD stroke means a wrong zero at boot. */
    public static final double kPlausibleLow = 0.85;
    public static final double kPlausibleHigh = 1.15;

    private final String m_name;
    private final double m_cadStroke;
    private double m_outStop = Double.NaN;
    private boolean m_lost = false;

    /**
     * @param name for alerts, for example "Talon 22 (right)"
     * @param cadStroke stroke from the CAD, meters; used until the out stop is measured
     */
    public RackSide(String name, double cadStroke) {
        m_name = name;
        m_cadStroke = cadStroke;
    }

    public boolean isLost() {
        return m_lost;
    }

    public boolean hasOutStop() {
        return !Double.isNaN(m_outStop);
    }

    /** The out stop: measured, or the CAD stroke until it is measured. */
    public double outStop() {
        return hasOutStop() ? m_outStop : m_cadStroke;
    }

    /** Where the stop of a move is: 0 or the out stop. */
    public double stopPosition(boolean out) {
        return out ? outStop() : 0.0;
    }

    /** Where the Motion Magic part of a move ends and the push starts. */
    public double approachGoal(boolean out) {
        return out ? outStop() - kApproachDistance : kApproachDistance;
    }

    /** At rest against a stop: has the side been pushed away from it? */
    public boolean awayFromStop(boolean out, double position) {
        return Math.abs(position - stopPosition(out)) > kPushedAway;
    }

    /** The Talon rebooted: the position it reads now is not true. */
    public void onReset() {
        m_lost = true;
    }

    /**
     * A push stopped at a hard stop.
     *
     * @param out true for the out stop
     * @param position the position the side reads now
     * @return the position to give the Talon (NaN = keep it) and an alert text (null = none)
     */
    public StopResult stalledAt(boolean out, double position) {
        boolean wasLost = m_lost;
        m_lost = false;
        if (out && !hasOutStop()) {
            // First time at the out stop: measure the stroke.
            if (position >= kPlausibleLow * m_cadStroke && position <= kPlausibleHigh * m_cadStroke) {
                m_outStop = position;
                return new StopResult(Double.NaN, null);
            }
            m_outStop = m_cadStroke;
            return new StopResult(m_cadStroke, "Intake " + m_name + " was not in at boot (out stop read "
                + String.format("%.3f", position) + " m): position set from the CAD stroke");
        }
        double stop = stopPosition(out);
        String alert = null;
        if (!wasLost && Math.abs(position - stop) > kSkipTolerance) {
            alert = String.format("Intake %s skipped teeth (%.1f cm off at the %s stop): position fixed",
                m_name, Math.abs(position - stop) * 100, out ? "out" : "in");
        }
        return new StopResult(stop, alert);
    }

    /**
     * A push did not reach a stop in time (blocked, or a motor problem).
     *
     * @return an alert text
     */
    public String pushTimedOut(boolean out) {
        return "Intake " + m_name + " did not reach the " + (out ? "out" : "in") + " stop: check for something in the way";
    }
}
