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
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import bline.commands.FollowPathV2;
import bline.field.FlippingUtil;
import bline.following.DriveType;
import bline.path.Path;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.util.AlertPublisher;
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
    private static final double kRotationP = 5.0; // rad/s per radian of heading error (1.0 turned 180 degrees in ~5 s; 3.0 trailed a turn by ~55 degrees)
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
    private final IntakeRack m_rack;
    private double m_routineTime = Double.NaN;
    private double m_lastShotStart = Double.NaN;
    /** Longest wait for the intake at the start: it takes ~0.45 s; this leaves room for a slow push. */
    private static final double kIntakeWait = 2.0;
    private final Alert m_noTrenchAlert = AlertPublisher.create("Health", "auto-no-trench",
        "Auto: the intake was not fully out at the start, so the robot did not drive under the trench;"
            + " it backed off and shoots from the alliance zone", Level.HIGH);

    public AutoRoutines(Drive drive, IntakeRack rack, IntakeRoller roller, Superstructure superstructure) {
        m_drive = drive;
        m_rack = rack;

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
         * Right trench, two bump trips (user, 2026-10-22). Start in the right trench lane, on the
         * starting line (a robot must start overlapping it: x 3.98-4.03), the intake facing the
         * alliance wall, 0.23 m short of the trench arm. The intake goes out first and the robot
         * waits for it: only with it out is the robot low enough for the trench. Back through the
         * trench (intake trailing), turn once clear of it, sweep the center line with the intake
         * leading, back over the right bump and shoot. Out over the bump again for a tighter
         * sweep, back over the bump, and shoot until auto ends. On each bump crossing the
         * "prepare" event spins up the flywheel and raises the hood, so the shot is ready when
         * the robot stops. The robot stops to shoot: shoot-on-the-move stays off until the time
         * of flight is measured.
         *
         * The robot stays off both center lines (user): never fully across the center line
         * (x 8.27) to the other alliance, and below the long center line (y 4.03), where our
         * partner's auto runs.
         */
        add("Right trench + bump x2", () -> trenchAndBump("right"));
        /*
         * The same route on the left (high-y) side: the right paths mirrored about the long
         * center line (tools/mirror-paths.py writes left-trench-*.json from right-trench-*.json;
         * edit the right ones and run it).
         */
        add("Left trench + bump x2", () -> trenchAndBump("left"));

        Tunables.publish("Auto Selection", m_chooser);
    }

    /**
     * A routine is made fresh each time from a supplier: in Commands v2 a command that is put in a
     * group cannot be used again, so a routine made once would only run once.
     *
     * <p>Requirements: a deferred command holds only the subsystems it is given, not those of the
     * commands inside it. The routine holds the drive (the path follower drives). Every other
     * command in it (intake, shoot) runs as a proxy ({@code asProxy()}): the scheduler starts it
     * on its own, so it holds its subsystems while it runs and a default command (the hood's
     * stow) cannot take them. The routine must not hold those subsystems itself: the path events
     * (intake, prepare) use them too, and an event that takes a subsystem from the routine would
     * cancel the whole routine.
     */
    private void add(String name, Supplier<Command> routine) {
        m_chooser.add(name, Commands.defer(routine, Set.of(m_drive))
            .beforeStarting(() -> {
                m_noTrenchAlert.set(false);
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

    /** Shoot (as a proxy: see {@link #add}) until auto ends. */
    private Command shoot() {
        return countShot(m_superstructure.shoot().asProxy());
    }

    /** Shoot for the dashboard's shoot time (no ball sensor). */
    private Command shootFor() {
        return countShot(m_superstructure.shoot().asProxy()
            .raceWith(Commands.defer(() -> Commands.waitSeconds(m_shootTime.get()), Set.of())));
    }

    private Command countShot(Command shoot) {
        return shoot.beforeStarting(() -> {
            m_shootEvents++;
            m_lastShotStart = Timer.getTimestamp() - m_routineStart;
        });
    }

    /**
     * The trench + bump route on one side ("right" or "left"): pose on the starting line, wait
     * for the intake (the robot fits under the trench only with it out), path 1, shoot, path 2,
     * shoot until auto ends.
     *
     * <p>If the intake is not fully out after {@link #kIntakeWait} (a side blocked, or a Talon
     * that does not answer), the robot must not go under the trench arm. It backs 0.65 m away
     * from the trench, along its lane (the fallback path), and shoots: the shot keeps the intake
     * going out and feeds once it is out, so the preload can still score.
     */
    private Command trenchAndBump(String side) {
        return Commands.sequence(
            startAt(side + "-trench-1"),
            m_rack.extend().asProxy()
                .andThen(Commands.waitUntil(m_rack::isExtended))
                .withTimeout(kIntakeWait),
            Commands.either(
                Commands.sequence(
                    follow(side + "-trench-1", false),
                    shootFor(),
                    follow(side + "-trench-2", false),
                    shoot()), // until auto ends
                Commands.sequence(
                    Commands.runOnce(() -> m_noTrenchAlert.set(true)),
                    follow(side + "-trench-fallback", false),
                    shoot()), // until auto ends
                m_rack::isExtended));
    }

    /** Set the pose to a path's first point (flipped for red), without moving. */
    private Command startAt(String pathName) {
        return Commands.runOnce(() -> {
            var path = new Path(pathName).setFlipped(Drive.isRedAlliance());
            m_drive.resetPose(path.getStartPose());
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
