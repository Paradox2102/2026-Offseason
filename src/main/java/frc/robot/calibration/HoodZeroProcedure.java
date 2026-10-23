package frc.robot.calibration;

import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

import frc.robot.subsystems.shooter.Hood;

/**
 * Hood zero: the hood's gear ratio and its angle at the lower stop.
 *
 * <ol>
 *   <li>A: home the hood to its lower stop.
 *   <li>D-pad up (held): the hood goes up slowly, past its soft limit, to the upper stop. The motor
 *       turns since the lower stop show.
 *   <li>Type the angle from the lower to the upper stop (measured, with an angle gauge) and the
 *       angle of the hood above level at its lower stop (for kG later).
 *   <li>X (or Capture): ratio = motor turns / hood turns. The Talon config is applied again and the
 *       hood homes again. Y: save.
 * </ol>
 */
public class HoodZeroProcedure extends Procedure {
    private final Calibration m_calibration;
    private final Hood m_hood;
    private final TunableDouble m_upperAngle = Tunables.addDouble("Calibration/Hood/Upper Stop Angle deg", 0);
    private final TunableDouble m_aboveLevel = Tunables.addDouble("Calibration/Hood/Lower Stop Above Level deg", 0);
    private String m_last = "";

    HoodZeroProcedure(Calibration calibration, Hood hood) {
        super("Hood zero");
        m_calibration = calibration;
        m_hood = hood;
        m_instructions = "A: home to the lower stop. D-pad up (hold): up slowly to the upper stop. Type the measured angle"
            + " from the lower to the upper stop, and the angle above level at the lower stop. X (or Capture): new ratio. Y: save";
    }

    @Override
    void bind(CommandGamepad pad, Trigger active) {
        pad.faceDown().and(active).onTrue(Commands.runOnce(this::home));
        pad.dpadUp().and(active).whileTrue(m_hood.jogUp());
        pad.faceLeft().and(active).onTrue(Commands.runOnce(this::capture));
    }

    /** Sim test: the typed angles, degrees. */
    public void setTyped(double upperStopDeg, double lowerStopAboveLevelDeg) {
        m_upperAngle.set(upperStopDeg);
        m_aboveLevel.set(lowerStopAboveLevelDeg);
    }

    public void home() {
        m_hood.rehome();
        m_last = "homing";
    }

    /** The new ratio from the motor turns and the typed angle. */
    @Override
    public void capture() {
        double upper = m_upperAngle.get();
        double turns = m_hood.motorTurnsFromStop();
        if (upper <= 1 || turns <= 0.5) {
            m_last = "type the upper stop angle and drive the hood up to its stop first";
            return;
        }
        double ratio = turns / (upper / 360);
        double aboveLevel = m_aboveLevel.get();
        m_hood.setCalibration(ratio, aboveLevel);
        var data = m_calibration.data();
        data.hoodRatio = ratio;
        data.hoodStopAboveLevelDeg = aboveLevel;
        m_last = String.format("ratio %.2f (%.2f motor turns over %.1f deg), lower stop %.1f deg above level: save with Y",
            ratio, turns, upper, aboveLevel);
    }

    @Override
    void periodic() {
        m_step = m_hood.isHomed() ? "Homed" : "Homing";
        m_readings[0] = String.format("Motor turns from the lower stop %.3f", m_hood.motorTurnsFromStop());
        m_readings[1] = String.format("Hood %.1f deg at ratio %.2f", m_hood.getAngleDeg(), m_hood.ratio());
        m_readings[2] = String.format("Typed: upper stop %.1f deg, lower stop %.1f deg above level", m_upperAngle.get(), m_aboveLevel.get());
        m_readings[3] = m_last;
    }
}
