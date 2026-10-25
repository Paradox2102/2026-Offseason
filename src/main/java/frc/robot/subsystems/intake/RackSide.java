package frc.robot.subsystems.intake;

/**
 * What one side of the intake rack knows about its own position. No hardware: unit tested.
 *
 * <p>The intake gets hit. A hit can push it in (the motor is backdriven: the reading follows) or
 * make a gear skip teeth on the rack (the reading is then wrong). One side can skip while the
 * other does not, and the frame flexes, so one side can also sit short while the other is out:
 * each side keeps its own knowledge, and trusts a reading only when the side stalls against a
 * hard stop. The motor's sensor cannot tell skipped teeth from something in the way, so the
 * direction of the error decides:
 * <ul>
 *   <li>At code start the reading means nothing. The rack is loose: while the robot is off it
 *       slides out down its slope, and a Talon reads 0 wherever it powers on. So the first move
 *       goes out (gravity helps), and the first out stall sets the reading to this side's out
 *       stop ("homed"). The first in stall after that within 3 cm of 0 sets the in stop (0) with
 *       no alert: the out stop constant may be a little off.
 *   <li>The motors never push into a stop (user, 2026-10-25; {@link RackMover}): only the out
 *       stop is ever reached, by gravity, with the motor coasting. Coming to rest there counts
 *       as a "stall" here. Within 1 cm of the stop it sets the reading to this side's measured
 *       out stop (IntakeConstants).
 *   <li>A stall 1 to 5 cm PAST the stop: a hard stop cannot be passed, so the reading is wrong
 *       (skipped teeth). Fixed, with an alert. More than 5 cm past: not trusted.
 *   <li>A stall more than 1 cm SHORT of the stop is never fixed: something may hold the side
 *       there, and then the reading is right. Fixing it would make a short side look out (too
 *       tall for the trench). A real skip of this kind is fixed at the other stop, where the
 *       reading is past it.
 *   <li>A side whose Talon rebooted ("lost") reads a position copied from the other side; the
 *       same rule applies to it, with no skip alert when it is fixed.
 * </ul>
 */
public class RackSide {
    /**
     * What to do after the side came to rest at a stop.
     *
     * @param setPosition the position to give the Talon (NaN = keep the reading)
     * @param alert an alert text, or null
     * @param atStop true if the side is now trusted to be at the stop
     */
    public record StopResult(double setPosition, String alert, boolean atStop) {}

    /** A stall within this of the expected stop is normal, meters. */
    public static final double kSkipTolerance = 0.01;
    /** A stall further than this past the expected stop is not trusted, meters. */
    public static final double kMaxCorrection = 0.05;
    /** The first in stall after homing within this of 0 is the in stop, meters. */
    public static final double kFirstInTolerance = 0.03;

    private final String m_name;
    private double m_outStop;
    private boolean m_lost = false;
    private boolean m_homed = false;
    /** Homed at the out stop; the in stop is not confirmed yet. */
    private boolean m_firstIn = false;

    /**
     * @param name for alerts, for example "Talon 22 (right)"
     * @param outStop this side's out stop, measured on the robot, meters
     */
    public RackSide(String name, double outStop) {
        m_name = name;
        m_outStop = outStop;
    }

    /** For alerts, for example "Talon 22 (right)". */
    public String name() {
        return m_name;
    }

    public boolean isLost() {
        return m_lost;
    }

    /** False until the first stall at the out stop after code start: until then the reading means nothing. */
    public boolean isHomed() {
        return m_homed;
    }

    /** The out stop, meters. */
    public double outStop() {
        return m_outStop;
    }

    /** A newly measured out stop (Calibration, "Intake zero"), meters. */
    public void setOutStop(double outStop) {
        m_outStop = outStop;
    }

    /** Where the stop of a move is: 0 or the out stop. */
    public double stopPosition(boolean out) {
        return out ? outStop() : 0.0;
    }

    /** The Talon rebooted: the position it reads now is not true. */
    public void onReset() {
        m_lost = true;
    }

    /**
     * The side came to rest at a stop (no longer a push: it coasts onto the out stop).
     *
     * @param out true for the out stop
     * @param position the position the side reads now
     */
    public StopResult stalledAt(boolean out, double position) {
        if (!m_homed && out) {
            // Homing: the stall is the out stop, whatever the reading says.
            m_homed = true;
            m_firstIn = true;
            m_lost = false;
            return new StopResult(outStop(), null, true);
        }
        if (!out && m_firstIn && Math.abs(position) <= kFirstInTolerance) {
            // The first in stop after homing: the out stop constant may be a little off.
            m_firstIn = false;
            m_lost = false;
            return new StopResult(0.0, null, true);
        }
        double stop = stopPosition(out);
        String where = out ? "out" : "in";
        // How far the reading is past the stop, in the direction of the move (negative = short).
        double past = out ? position - stop : stop - position;
        if (past < -kSkipTolerance) {
            return new StopResult(Double.NaN, String.format(
                "Intake %s stopped %.1f cm short of the %s stop: something holds it, or it skipped teeth."
                    + " Move it to the other stop and back to check again",
                m_name, -past * 100, where), false);
        }
        if (past > kMaxCorrection) {
            return new StopResult(Double.NaN, String.format(
                "Intake %s read %.1f cm past the %s stop: the reading is wrong; check the rack and its gear",
                m_name, past * 100, where), false);
        }
        String alert = past > kSkipTolerance && !m_lost
            ? String.format("Intake %s skipped teeth (%.1f cm past the %s stop): position fixed", m_name, past * 100, where)
            : null;
        m_lost = false;
        return new StopResult(stop, alert, true);
    }
}
