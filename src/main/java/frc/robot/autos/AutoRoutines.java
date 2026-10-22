package frc.robot.autos;

import java.util.function.Supplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.controller.PIDController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

import bline.commands.FollowPathV2;
import bline.field.FlippingUtil;
import bline.following.DriveType;
import bline.path.Path;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

/**
 * The autonomous routines and the dashboard chooser ("Auto Selection").
 *
 * <p>BLine follows a path made in the BLine editor (files in deploy/autos/paths). It sends a
 * robot-relative speed every loop; {@link Drive#driveRobotRelative} drives it closed loop.
 */
public class AutoRoutines {
    /*
     * BLine gains, from the BLine tuning guide. Tune in this order, one at a time, with the other
     * two at 0: translation on a 5 m straight path, rotation on a 0-to-90 degree path, then
     * cross-track with the robot started 0.25 m off the line and no pose reset.
     * Each controller has its own object: BLine changes them (continuous input, tolerance, reset),
     * so they must not be shared with other code.
     */
    private static final double kTranslationP = 2.0; // m/s per meter of path left
    private static final double kRotationP = 1.0; // rad/s per radian of heading error
    private static final double kCrossTrackP = 0.2; // m/s per meter off the line (6.0 made the robot snake)

    private final Drive m_drive;
    private final FollowPathV2.Builder m_builder;
    private final Selectable<Command> m_chooser = new Selectable<>();

    public AutoRoutines(Drive drive) {
        m_drive = drive;

        // BLine flips a red path by turning it 180 degrees about the field center. Use the size
        // of the field we play on (its default is the welded field, 2-3 cm bigger than AndyMark).
        FlippingUtil.fieldSizeX = FieldConstants.kFieldLength;
        FlippingUtil.fieldSizeY = FieldConstants.kFieldWidth;

        m_builder = new FollowPathV2.Builder(
            DriveType.SWERVE,
            drive,
            drive::getPose, // live: BLine reads the pose right after its reset
            drive::resetPose,
            drive::getRobotVelocity, // measured, robot-relative
            drive::driveRobotRelative,
            new PIDController(kTranslationP, 0, 0),
            new PIDController(kRotationP, 0, 0),
            new PIDController(kCrossTrackP, 0, 0))
            .withDefaultShouldFlip()
            .withTelemetry(Telemetry.getTable("Auto/BLine"));

        m_chooser.addDefault("Do nothing", Commands.none().withName("Do nothing"));
        /*
         * "first-straight" starts at the field corner (0, 0). With the pose reset, the robot
         * thinks it is at the corner, which is right only in the shop with no tags in view. On a
         * field the cameras correct the pose at once and BLine then drives to the real point
         * (2.0, 0.8) near the blue corner, wherever the robot is. Make match paths in the BLine
         * editor from the real start position.
         */
        add("B-Line Test (shop only, no tags)", () -> follow("first-straight", true));

        Tunables.publish("Auto Selection", m_chooser);
    }

    /**
     * A routine is made fresh each time from a supplier: in Commands v2 a command that is put in a
     * group cannot be used again, so a routine made once would only run once.
     */
    private void add(String name, Supplier<Command> routine) {
        m_chooser.add(name, Commands.defer(routine, java.util.Set.of(m_drive)).withName(name));
    }

    /** Follow one path. {@code resetPose}: start by setting the pose to the path's first point. */
    private Command follow(String pathName, boolean resetPose) {
        var command = m_builder.build(new Path(pathName));
        return resetPose ? command.withPoseReset() : command;
    }

    /** The auto used when nobody picks one on the dashboard. */
    public void setDefault(String name) {
        m_chooser.setDefault(name);
    }

    public Command getSelected() {
        return m_chooser.getSelected();
    }
}
