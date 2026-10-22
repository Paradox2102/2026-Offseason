package frc.robot.autos;

import java.util.Set;
import java.util.function.Supplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.controller.PIDController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.system.Timer;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

import bline.commands.FollowPathV2;
import bline.field.FlippingUtil;
import bline.following.DriveType;
import bline.path.Path;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeRack;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.superstructure.Superstructure;

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
    private static final double kRotationP = 3.0; // rad/s per radian of heading error (1.0 turned 180 degrees in ~5 s)
    private static final double kCrossTrackP = 0.2; // m/s per meter off the line (6.0 made the robot snake)

    private final Drive m_drive;
    private final FollowPathV2.Builder m_builder;
    private final Selectable<Command> m_chooser = new Selectable<>();

    /** No ball sensor: a shoot event feeds this long, seconds. */
    private final TunableDouble m_shootTime = Tunables.addDouble("Auto/Shoot Time s", 2.5);
    private final Command m_intakeEvent;
    private final Command m_prepareEvent;
    private final Superstructure m_superstructure;
    private int m_intakeEvents = 0;
    private int m_shootEvents = 0;
    private double m_routineStart = Double.NaN;
    private double m_routineTime = Double.NaN;
    private double m_lastShotStart = Double.NaN;

    public AutoRoutines(Drive drive, IntakeRack rack, IntakeRoller roller, Superstructure superstructure) {
        m_drive = drive;

        /*
         * Events: a path can name an event at a point along it ("event_trigger" in the path file).
         * When the robot passes that point, BLine starts the command registered for that name.
         * The command does not stop when the path ends, so cancelEvents() stops them at the end
         * of auto.
         */
        m_intakeEvent = Commands.sequence(rack.extend(), roller.runIn())
            .beforeStarting(() -> m_intakeEvents++).withName("Auto Intake");
        m_prepareEvent = superstructure.prepare().withName("Auto Prepare");
        m_superstructure = superstructure;
        FollowPathV2.registerEventTrigger("intake", m_intakeEvent);
        FollowPathV2.registerEventTrigger("prepare", m_prepareEvent);

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
         * Right trench, two bump trips (user, 2026-10-22). Start on the starting line at
         * (3.51, 2.66) facing the neutral zone. The intake goes out at once: out, the rack sits
         * lower, which it must to fit under the trench. Through the right trench, sweep the
         * center line, back over the right bump and shoot. Out over the bump again for a tighter
         * sweep, back over the bump, and shoot until auto ends. On each bump crossing the
         * "prepare" event spins up the flywheel and raises the hood, so the shot is ready when
         * the robot stops. The robot stops to shoot: shoot-on-the-move stays off until the time
         * of flight is measured.
         */
        add("Right trench + bump x2", () -> Commands.sequence(
            follow("right-trench-1", true),
            shootFor(),
            follow("right-trench-2", false),
            countShot(m_superstructure.shoot()))); // until auto ends

        Tunables.publish("Auto Selection", m_chooser);
    }

    /**
     * A routine is made fresh each time from a supplier: in Commands v2 a command that is put in a
     * group cannot be used again, so a routine made once would only run once.
     */
    private void add(String name, Supplier<Command> routine) {
        m_chooser.add(name, Commands.defer(routine, Set.of(m_drive))
            .beforeStarting(() -> {
                m_routineStart = Timer.getTimestamp();
                m_routineTime = Double.NaN;
                m_intakeEvents = 0;
                m_shootEvents = 0;
                m_lastShotStart = Double.NaN;
            })
            .finallyDo(() -> m_routineTime = Timer.getTimestamp() - m_routineStart)
            .withName(name));
    }

    /** Stop the event commands (intake, shoot) that are still running. Call when auto ends. */
    public void cancelEvents() {
        m_intakeEvent.cancel();
        m_prepareEvent.cancel();
    }

    /** Call every loop. */
    public void periodic() {
        Telemetry.log("Auto/Routine Time s", m_routineTime);
        Telemetry.log("Auto/Intake Events", m_intakeEvents);
        Telemetry.log("Auto/Shots", m_shootEvents);
        Telemetry.log("Auto/Last Shot Start s", m_lastShotStart);
    }

    /** For the sim test: how long the last routine took (NaN while one runs), and the event counts. */
    public String status() {
        return String.format("routine %.1f s, %d intake events, %d shots, last shot started at %.1f s",
            m_routineTime, m_intakeEvents, m_shootEvents, m_lastShotStart);
    }

    /** Shoot for the dashboard's shoot time (no ball sensor). */
    private Command shootFor() {
        return countShot(m_superstructure.shoot()
            .raceWith(Commands.defer(() -> Commands.waitSeconds(m_shootTime.get()), Set.of())));
    }

    private Command countShot(Command shoot) {
        return shoot.beforeStarting(() -> {
            m_shootEvents++;
            m_lastShotStart = Timer.getTimestamp() - m_routineStart;
        });
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
