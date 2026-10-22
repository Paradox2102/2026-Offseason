package frc.robot.sim;

import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Timer;

import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

/**
 * A scripted simulator run with no driver station, for {@code tools/sim-smoke.sh}.
 *
 * <p>Runs only when the environment variable SIM_SMOKE=1. It drives teleop with a fake stick,
 * runs the "B-Line Test" auto, kicks the pose to see vision pull it back, prints a summary
 * ("SMOKE ..." lines) and exits. Any exception in the robot code shows in the output, which the
 * script checks.
 */
public class SimSmokeTest {
    private final RobotContainer m_container;
    private final Drive m_drive;
    // Scripted sticks. (DriverStationSim joystick axes did not reach Gamepad in a headless run,
    // so the teleop command gets these values directly; the GUI sim with a real gamepad works.)
    private double m_leftY = 0;
    private double m_leftX = 0;
    private double m_rightX = 0;
    private final Timer m_timer = new Timer();
    private int m_step = -1;
    private double m_maxTeleopSpeed = 0;
    private double m_headingAtRelease = 0;
    private boolean m_teleopScheduled = false;

    public SimSmokeTest(RobotContainer container, Drive drive) {
        m_container = container;
        m_drive = drive;
        m_container.selectAuto("B-Line Test");
        // Start in the blue zone, where the cameras can see tags.
        m_drive.resetPose(new Pose2d(3.0, 2.0, Rotation2d.ZERO));
        DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        m_timer.start();
    }

    public void periodic() {
        double t = m_timer.get();
        if (t < 1) {
            step(0, "disabled", RobotMode.TELEOPERATED, false);
        } else if (t < 3) {
            step(1, "teleop: drive forward and left", RobotMode.TELEOPERATED, true);
            m_leftY = -0.8; // pushed away from the driver
            m_leftX = -0.3; // pushed left
            m_rightX = 0;
        } else if (t < 4) {
            step(10, "teleop: slow down", RobotMode.TELEOPERATED, true);
            m_leftY = -0.3;
            m_leftX = 0;
        } else if (t < 6) {
            step(2, "teleop: turn while driving", RobotMode.TELEOPERATED, true);
            m_rightX = -0.6; // turn left (counterclockwise)
        } else if (t < 9) {
            step(3, "teleop: stick released, heading hold", RobotMode.TELEOPERATED, true);
            m_rightX = 0;
        } else if (t < 10) {
            step(4, "disabled", RobotMode.TELEOPERATED, false);
            m_leftY = 0;
            m_leftX = 0;
        } else if (t < 22) {
            step(5, "auto: B-Line Test", RobotMode.AUTONOMOUS, true);
        } else if (t < 23) {
            step(6, "disabled, kick pose", RobotMode.AUTONOMOUS, false);
        } else if (t < 30) {
            step(7, "disabled, vision corrects", RobotMode.AUTONOMOUS, false);
        } else {
            report("end");
            writeTopicList();
            System.exit(0);
        }
        // The scheduler drops a command scheduled while disabled, so wait for the enable.
        if (m_step == 1 && !m_teleopScheduled && RobotState.isEnabled()) {
            CommandScheduler.getInstance().schedule(DriveCommands.teleop(
                m_drive, () -> m_leftY, () -> m_leftX, () -> m_rightX, () -> false));
            m_teleopScheduled = true;
        }
        if ((m_step >= 1 && m_step <= 3) || m_step == 10) {
            var v = m_drive.getRobotVelocity();
            m_maxTeleopSpeed = Math.max(m_maxTeleopSpeed, Math.hypot(v.vx, v.vy));
        }
        DriverStationSim.notifyNewData();
    }

    private void step(int step, String name, RobotMode mode, boolean enabled) {
        if (step == m_step) {
            return;
        }
        if (m_step >= 0) {
            report("end of step " + m_step);
        }
        m_step = step;
        System.out.println("SMOKE step " + step + ": " + name);
        DriverStationSim.setRobotMode(mode);
        DriverStationSim.setEnabled(enabled);
        if (step == 3) {
            m_headingAtRelease = m_drive.getHeading().getDegrees();
        }
        if (step == 4) {
            System.out.printf("SMOKE heading hold: %.1f deg at release, %.1f deg after 3 s%n",
                m_headingAtRelease, m_drive.getHeading().getDegrees());
        }
        if (step == 6) {
            m_drive.getSim().kickPose();
            report("after kick");
        }
    }

    /** Every NetworkTables topic, one per line, so the script can check the Elastic layout. */
    private void writeTopicList() {
        var names = java.util.Arrays.stream(NetworkTableInstance.getDefault().getTopics())
            .map(topic -> topic.getName()).sorted().toList();
        try {
            java.nio.file.Files.write(java.nio.file.Path.of("build", "sim-topics.txt"), names);
        } catch (java.io.IOException e) {
            System.out.println("SMOKE could not write the topic list: " + e);
        }
    }

    private void report(String when) {
        var pose = m_drive.getPose();
        var truth = m_drive.getSim().truthPose();
        System.out.printf("SMOKE %s: pose (%.2f, %.2f, %.1f deg) truth (%.2f, %.2f, %.1f deg) error %.3f m, max teleop speed %.2f m/s%n",
            when, pose.getX(), pose.getY(), pose.getRotation().getDegrees(),
            truth.getX(), truth.getY(), truth.getRotation().getDegrees(),
            pose.getTranslation().getDistance(truth.getTranslation()), m_maxTeleopSpeed);
    }
}
