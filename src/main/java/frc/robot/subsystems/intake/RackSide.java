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
 *   <li>A stall 1 to 5 cm from the expected stop means skipped teeth: fixed, with an alert.
 *   <li>A stall more than 5 cm away is not a skip: something blocks the rack. The reading is not
 *       changed (re-truing it would make it wrong by the whole distance).
 *   <li>A side whose Talon rebooted ("lost") reads a position copied from the other side; its
 *       stall always re-trues it, with no alert.
 * </ul>
 */
public class RackSide {
    /**
     * What to do after a push stopped.
     *
     * @param setPosition the position to give the Talon (NaN = keep the reading)
     * @param alert an alert text, or null
     * @param atStop true if the side is now trusted to be at the stop
     */
    public record StopResult(double setPosition, String alert, boolean atStop) {}

    /** The push starts this far before the stop, meters. */
    public static final double kApproachDistance = 0.035;
    /** A stall further than this from the expected stop means skipped teeth, meters. */
    public static final double kSkipTolerance = 0.01;
    /** A stall further than this from the expected stop is not a skip: the rack is blocked, meters. */
    public static final double kMaxCorrection = 0.05;
    /** Further than this from its stop at rest, the side was pushed away and goes back, meters. */
    public static final double kPushedAway = 0.02;
    /**
     * The first out stop is measured only within these fractions of the CAD stroke (about +-2 cm,
     * less than the 3.5 cm approach).
     */
    public static final double kPlausibleLow = 0.94;
    public static final double kPlausibleHigh = 1.06;

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
     * A push stopped moving.
     *
     * @param out true for a push toward the out stop
     * @param position the position the side reads now
     */
    public StopResult stalledAt(boolean out, double position) {
        if (m_lost) {
            // The reading was copied from the other side: trust the stop, measure nothing.
            m_lost = false;
            return new StopResult(stopPosition(out), null, true);
        }
        double stop = stopPosition(out);
        double error = Math.abs(position - stop);
        String where = out ? "out" : "in";
        if (error > kMaxCorrection) {
            return new StopResult(Double.NaN, String.format(
                "Intake %s stopped %.1f cm before the %s stop: something blocks it (or it was not in at boot)",
                m_name, error * 100, where), false);
        }
        if (out && !hasOutStop()) {
            // First time at the out stop: measure the stroke.
            if (position >= kPlausibleLow * m_cadStroke && position <= kPlausibleHigh * m_cadStroke) {
                m_outStop = position;
                return new StopResult(Double.NaN, null, true);
            }
            return new StopResult(m_cadStroke, String.format("Intake %s out stop read %.3f m, expected about %.3f:"
                + " position set from the CAD stroke; the next extend measures again", m_name, position, m_cadStroke), true);
        }
        String alert = error > kSkipTolerance
            ? String.format("Intake %s skipped teeth (%.1f cm off at the %s stop): position fixed", m_name, error * 100, where)
            : null;
        return new StopResult(stop, alert, true);
    }

    /**
     * A push did not stop in time (a motor problem, or the rack moves freely past where a stop
     * should be).
     *
     * @return an alert text
     */
    public String pushTimedOut(boolean out) {
        return "Intake " + m_name + " did not reach the " + (out ? "out" : "in") + " stop: check for something in the way";
    }
}
