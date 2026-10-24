package frc.robot.calibration;

import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.RobotState;

import frc.robot.subsystems.shooter.Turret;

/**
 * Turret zero: the jig zero, and an optional trim. For the trim, jog the turret until the shooter
 * lines up with a mark at the jig angle (straight to the robot's right), and record: from then on
 * the turret aims as if that reading were 0. Used at once.
 *
 * <p>After the jig zero (enabled), the turret turns 90 degrees clockwise seen from above, so the
 * shooter points to the robot's back: this shows that the zero took, the direction and the amount.
 *
 * <p>Test pad: A jig zero (turret in its jig), D-pad up back to the jig angle, D-pad down the 90
 * degree check again, D-pad left/right jog (bumper held: bigger steps), X record the trim, B clear
 * the trim, Y save. While this procedure is picked the turret holds the jogged angle instead of
 * aiming.
 */
public class TurretZeroProcedure extends Procedure {
    private static final double kStepDeg = 0.25;
    private static final double kBigStepDeg = 2.0;
    /** The check turn after the jig zero: 90 degrees clockwise seen from above. */
    private static final double kCheckDeg = -90.0;
    private final Calibration m_calibration;
    private final Turret m_turret;
    /** The jogged goal, degrees from the jig (with the trim in use). */
    private double m_goalDeg = 0;
    private String m_last = "";
    /** The jig zero was asked for while enabled: turn for the check once the Talon takes it. */
    private boolean m_checkPending = false;

    TurretZeroProcedure(Calibration calibration, Turret turret) {
        super("Turret zero");
        m_calibration = calibration;
        m_turret = turret;
        m_instructions = "Enabled, turret in its jig: A (or Capture) zeroes it, then it turns 90 deg clockwise (shooter to the"
            + " robot's back). D-pad up: back to the jig angle; D-pad down: the 90 deg check again. Trim: D-pad left/right"
            + " turns it (bumper: bigger steps) until the shooter lines up with the mark at the jig angle; X records the trim,"
            + " B clears it. Y: save";
    }

    @Override
    void bind(CommandGamepad pad, Trigger active) {
        // isUtility() is true while disabled too; a command scheduled while disabled is dropped.
        active.and(RobotState::isEnabled).whileTrue(m_turret.track(() -> m_goalDeg / 360).beforeStarting(() -> m_goalDeg = m_turret.getPosition() * 360)
            .withName("Calibration Turret Hold"));
        pad.faceDown().and(active).onTrue(Commands.runOnce(this::capture).ignoringDisable(true));
        pad.dpadLeft().and(active).onTrue(Commands.runOnce(() -> jog(pad.rightBumper().getAsBoolean() ? kBigStepDeg : kStepDeg)));
        pad.dpadRight().and(active).onTrue(Commands.runOnce(() -> jog(pad.rightBumper().getAsBoolean() ? -kBigStepDeg : -kStepDeg)));
        pad.dpadUp().and(active).onTrue(Commands.runOnce(this::toJig));
        pad.dpadDown().and(active).onTrue(Commands.runOnce(this::checkTurn));
        pad.faceLeft().and(active).onTrue(Commands.runOnce(this::recordTrim));
        pad.faceRight().and(active).onTrue(Commands.runOnce(this::clearTrim));
    }

    /** The jig zero (the turret must be in its jig). */
    @Override
    public void capture() {
        CommandScheduler.getInstance().schedule(m_turret.zeroInJig());
        m_goalDeg = 0;
        m_checkPending = RobotState.isEnabled();
        m_last = m_checkPending ? "jig zero asked for: the 90 deg check turn follows" : "jig zero asked for (disabled: no check turn)";
    }

    /** Back to the jig angle (with the trim in use: the mark). */
    public void toJig() {
        m_checkPending = false;
        m_goalDeg = 0;
        m_last = "back to the jig angle";
    }

    /** 90 degrees clockwise from the jig: the shooter points to the robot's back. */
    public void checkTurn() {
        m_checkPending = false;
        m_goalDeg = kCheckDeg;
        m_last = "check: 90 deg clockwise, the shooter must point to the robot's back";
    }

    /** Turn the held goal by this much, degrees (positive = counterclockwise). */
    public void jog(double deg) {
        m_checkPending = false;
        m_goalDeg += deg;
    }

    /** The turret points at the mark now: that reading is the new 0. */
    public void recordTrim() {
        double trimDeg = (m_turret.getPosition() + m_turret.getTrim()) * 360;
        m_turret.setTrim(trimDeg / 360);
        m_calibration.data().turretTrimDeg = trimDeg;
        m_goalDeg = 0;
        m_last = String.format("trim %.2f deg recorded", trimDeg);
    }

    public void clearTrim() {
        m_turret.setTrim(0);
        m_calibration.data().turretTrimDeg = 0;
        m_goalDeg = m_turret.getPosition() * 360;
        m_last = "trim cleared";
    }

    @Override
    void periodic() {
        if (m_checkPending && m_turret.isZeroed() && !m_turret.isJigZeroPending()) {
            checkTurn();
        }
        m_step = m_turret.isZeroed() ? "Zeroed: trim if needed" : "Not zeroed: put the turret in its jig and press A";
        m_readings[0] = String.format("Turret %.2f deg (goal %.2f)", m_turret.getPosition() * 360, m_goalDeg);
        m_readings[1] = String.format("Trim in use %.2f deg", m_turret.getTrim() * 360);
        m_readings[2] = m_last;
        m_readings[3] = "";
    }

    /** This procedure holds and moves the turret itself. */
    @Override
    boolean holdTurret() {
        return false;
    }
}
