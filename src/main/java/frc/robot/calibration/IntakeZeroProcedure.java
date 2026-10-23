package frc.robot.calibration;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;

import frc.robot.subsystems.intake.IntakeRack;

/**
 * Intake zero: the out stop of each side. The rack goes out and in three times; each side keeps
 * its largest stall reading at the out stop (the reading before any fix). Those become the out
 * stops at once. Start with the intake fully in and nothing in its way.
 *
 * <p>Test pad: A start (again), B stop, Y save.
 */
public class IntakeZeroProcedure extends Procedure {
    private static final int kCycles = 3;
    private final Calibration m_calibration;
    private final IntakeRack m_rack;
    private final Command m_run;
    private int m_cycle = 0;
    private double m_maxLeft = Double.NaN;
    private double m_maxRight = Double.NaN;
    private String m_last = "";

    IntakeZeroProcedure(Calibration calibration, IntakeRack rack) {
        super("Intake zero");
        m_calibration = calibration;
        m_rack = rack;
        m_instructions = "Intake fully in, nothing in its way, robot on blocks or with room in front. A (or Capture): out and in"
            + " three times; each side's largest stall reading becomes its out stop. B: stop. Y: save";
        var cycles = new Command[kCycles];
        for (int i = 0; i < kCycles; i++) {
            cycles[i] = Commands.sequence(
                Commands.runOnce(() -> m_cycle++),
                rack.extend(),
                Commands.runOnce(this::readStalls),
                rack.retract());
        }
        m_run = Commands.sequence(
            Commands.runOnce(() -> {
                m_cycle = 0;
                m_maxLeft = Double.NaN;
                m_maxRight = Double.NaN;
                m_last = "running";
            }),
            rack.retract(), // from the in stop, re-trued there
            Commands.sequence(cycles),
            Commands.runOnce(this::apply))
            .withName("Calibration Intake Zero");
    }

    @Override
    void bind(CommandGamepad pad, Trigger active) {
        pad.faceDown().and(active).onTrue(Commands.runOnce(this::capture));
        pad.faceRight().and(active).onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancel(m_run)));
        // Another procedure picked, or out of Utility mode: stop.
        active.onFalse(Commands.runOnce(() -> CommandScheduler.getInstance().cancel(m_run)));
    }

    /** Start the three cycles. */
    @Override
    public void capture() {
        CommandScheduler.getInstance().schedule(m_run);
    }

    public boolean isRunning() {
        return m_run.isScheduled();
    }

    private void readStalls() {
        m_maxLeft = max(m_maxLeft, m_rack.lastStall(true));
        m_maxRight = max(m_maxRight, m_rack.lastStall(false));
    }

    private static double max(double a, double b) {
        return Double.isNaN(a) ? b : Double.isNaN(b) ? a : Math.max(a, b);
    }

    /** The new out stops, used at once. */
    private void apply() {
        if (Double.isNaN(m_maxLeft) || Double.isNaN(m_maxRight)) {
            m_last = "no stall read on a side: nothing changed";
            return;
        }
        m_rack.setOutStops(m_maxLeft, m_maxRight);
        var data = m_calibration.data();
        data.intakeOutStopLeft = m_maxLeft;
        data.intakeOutStopRight = m_maxRight;
        m_last = String.format("out stops now %.4f m (left), %.4f m (right): save with Y", m_maxLeft, m_maxRight);
    }

    @Override
    void periodic() {
        m_step = isRunning() ? "Cycle " + m_cycle + " of " + kCycles : "Press A to start";
        m_readings[0] = String.format("Last stall: left %.4f m, right %.4f m", m_rack.lastStall(true), m_rack.lastStall(false));
        m_readings[1] = String.format("Largest: left %.4f m, right %.4f m", m_maxLeft, m_maxRight);
        m_readings[2] = String.format("Out stops in use: left %.4f m, right %.4f m", m_rack.outStop(true), m_rack.outStop(false));
        m_readings[3] = m_last;
    }
}
