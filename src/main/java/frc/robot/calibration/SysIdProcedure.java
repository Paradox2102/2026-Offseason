package frc.robot.calibration;

import java.util.Set;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import org.wpilib.command2.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.drive.Drive;

/**
 * Drive SysId on blocks or on carpet: bumpers pick the routine (translation, steer, rotation),
 * A/B quasistatic forward/reverse, X/Y dynamic forward/reverse (held). The dashboard's Tuning tab
 * runs the same tests.
 */
public class SysIdProcedure extends Procedure {
    private final Drive m_drive;
    private Drive.Routine m_routine = Drive.Routine.TRANSLATION;

    SysIdProcedure(Drive drive) {
        super("Drive SysId");
        m_drive = drive;
        m_instructions = "Bumpers: routine. Hold A/B: quasistatic forward/reverse; X/Y: dynamic forward/reverse."
            + " Release to stop. Logs go to the hoot log";
    }

    @Override
    void bind(CommandGamepad pad, Trigger active) {
        var routines = Drive.Routine.values();
        pad.leftBumper().and(active).onTrue(Commands.runOnce(
            () -> m_routine = routines[(m_routine.ordinal() + routines.length - 1) % routines.length]).ignoringDisable(true));
        pad.rightBumper().and(active).onTrue(Commands.runOnce(
            () -> m_routine = routines[(m_routine.ordinal() + 1) % routines.length]).ignoringDisable(true));
        pad.faceDown().and(active).whileTrue(selected(Direction.FORWARD, false));
        pad.faceRight().and(active).whileTrue(selected(Direction.REVERSE, false));
        pad.faceLeft().and(active).whileTrue(selected(Direction.FORWARD, true));
        pad.faceUp().and(active).whileTrue(selected(Direction.REVERSE, true));
    }

    /** Built when pressed, so it uses the routine picked at that moment. */
    private Command selected(Direction direction, boolean dynamic) {
        return Commands.defer(() -> m_drive.sysId(m_routine, direction, dynamic), Set.of(m_drive));
    }

    public Drive.Routine routine() {
        return m_routine;
    }

    @Override
    boolean saveWithY() {
        return false;
    }

    @Override
    void periodic() {
        m_step = "Routine: " + m_routine.name();
        m_readings[0] = "";
        m_readings[1] = "";
        m_readings[2] = "";
        m_readings[3] = "";
    }
}
