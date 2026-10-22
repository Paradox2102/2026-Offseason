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
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.intake.IntakeConstants;
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
    /** SIM_SMOKE_ALLIANCE=red: run the auto as red (teleop and the shooting step stay blue). */
    private static final boolean kRedAuto = "red".equalsIgnoreCase(System.getenv("SIM_SMOKE_ALLIANCE"));
    /** SIM_SMOKE_STAY=<seconds>: keep the sim running after the script, to look at it in AdvantageScope. */
    private static final double kStaySeconds = parseStay();
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
    private org.wpilib.command2.Command m_pendingIntake = null;
    private double m_intakeStart = Double.NaN;
    private double m_secondExtendTime = Double.NaN;
    private boolean m_hitDone = false;
    private boolean m_blockReported = false;
    private boolean m_autoStartReported = false;
    private boolean m_trenchReported = false;
    private double m_autoMinY = Double.POSITIVE_INFINITY;
    private double m_autoMaxX = Double.NEGATIVE_INFINITY;
    private double m_autoMinYX = Double.NaN;
    private double m_autoMinYT = Double.NaN;
    private double m_autoFeedTime = 0;
    private double m_leadSum = 0;
    private double m_leadTime = 0;
    private double m_leadMax = 0;
    private String m_leadMaxAt = "";
    private double m_readyTime = Double.NaN;
    private boolean m_fed = false;
    private double m_lastFlywheel = 0;
    private double m_autoMaxZ = 0;
    private double m_autoMaxPitchDeg = 0;
    private double m_autoMaxTruthError = 0;

    public SimSmokeTest(RobotContainer container, Drive drive) {
        m_container = container;
        m_drive = drive;
        m_container.selectAuto("Right trench + bump x2");
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
        } else if (t < 11) {
            step(11, "teleop: intake retract", RobotMode.TELEOPERATED, true);
            m_leftY = 0;
            m_leftX = 0;
        } else if (t < 16) {
            step(12, "teleop: intake extend, left side blocked at 0.15 m for 2 s", RobotMode.TELEOPERATED, true);
            m_container.getIntakeRack().simBlockLeft(t < 13 ? 0.15 : Double.NaN);
        } else if (t < 20) {
            step(13, "teleop: shoot from (2.5, 3.0) in the blue zone", RobotMode.TELEOPERATED, true);
        } else if (t < 21) {
            step(4, kRedAuto ? "disabled, alliance set to red" : "disabled", RobotMode.TELEOPERATED, false);
            if (kRedAuto) {
                DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
            }
        } else if (t < 41) {
            step(5, "auto: Right trench + bump x2 (20 s)", RobotMode.AUTONOMOUS, true);
        } else if (t < 42) {
            step(6, "disabled, kick pose", RobotMode.AUTONOMOUS, false);
        } else if (t < 49) {
            step(7, "disabled, vision corrects", RobotMode.AUTONOMOUS, false);
        } else if (m_step != 99) {
            step(99, "script done", RobotMode.TELEOPERATED, false);
            report("end");
            writeTopicList();
            if (t >= 49 + kStaySeconds) {
                System.exit(0);
            }
            System.out.println("SMOKE staying up for " + kStaySeconds + " s (SIM_SMOKE_STAY) for AdvantageScope");
        } else if (t >= 49 + kStaySeconds) {
            System.exit(0);
        }
        // The scheduler drops a command scheduled while disabled, so wait for the enable.
        if (m_step == 1 && !m_teleopScheduled && RobotState.isEnabled()) {
            CommandScheduler.getInstance().schedule(DriveCommands.teleop(
                m_drive, () -> m_leftY, () -> m_leftX, () -> m_rightX, () -> false));
            m_teleopScheduled = true;
        }
        if (m_pendingIntake != null && RobotState.isEnabled()) {
            CommandScheduler.getInstance().schedule(m_pendingIntake);
            m_pendingIntake = null;
            m_intakeStart = t;
        }
        var rack = m_container.getIntakeRack();
        if (m_step == 3 && Double.isNaN(m_secondExtendTime) && rack.getPosition() > IntakeConstants.kRackStroke - 0.01) {
            m_secondExtendTime = t - m_intakeStart;
        }
        if (m_step == 13) {
            var sup = m_container.getSuperstructure();
            if (Double.isNaN(m_readyTime) && sup.isReady()) {
                m_readyTime = t - 16;
            }
            m_fed |= m_container.getFeeder().isFeeding();
            m_lastFlywheel = m_container.getFlywheel().getSpeed();
        }
        // The rack must be out (lower) before the robot reaches the trench. The auto checks use the
        // blue frame: a red pose is turned 180 degrees about the field center first.
        var pose = toBlueFrame(m_drive.getPose());
        // From the start pose on (the first loop of the auto still has the teleop pose).
        if (m_step == 5 && m_autoStartReported) {
            if (m_container.getFeeder().isFeeding()) {
                m_autoFeedTime += 0.020;
            }
            // Over the bump: how high and how steep, and how far odometry runs from the truth.
            var truth3d = m_drive.getSim().truthPose3d();
            m_autoMaxZ = Math.max(m_autoMaxZ, truth3d.getZ());
            m_autoMaxPitchDeg = Math.max(m_autoMaxPitchDeg, Math.toDegrees(Math.abs(truth3d.getRotation().getY())));
            m_autoMaxTruthError = Math.max(m_autoMaxTruthError,
                m_drive.getPose().getTranslation().getDistance(m_drive.getSim().truthPose().getTranslation()));
            // Intake leading: in the neutral zone, while moving, how far the robot front (the
            // intake) points from the direction of travel.
            var v = m_drive.getFieldVelocity();
            double speed = Math.hypot(v.vx, v.vy);
            double vxBlue = kRedAuto ? -v.vx : v.vx;
            double vyBlue = kRedAuto ? -v.vy : v.vy;
            // Only the collecting part: out and along the center line, not the drive back
            // (there the intake trails on purpose).
            if (pose.getX() > 5.9 && speed > 0.5 && vxBlue > -0.5 * speed) {
                double travel = Math.atan2(vyBlue, vxBlue);
                double off = Math.abs(Math.toDegrees(Math.IEEEremainder(pose.getRotation().getRadians() - travel, 2 * Math.PI)));
                m_leadSum += off * 0.020;
                m_leadTime += 0.020;
                if (off > m_leadMax) {
                    m_leadMax = off;
                    m_leadMaxAt = String.format("(%.2f, %.2f) heading %.0f, travel %.0f, %.1f m/s, t %.1f s",
                        pose.getX(), pose.getY(), pose.getRotation().getDegrees(), Math.toDegrees(travel), speed, t - 21);
                }
            }
            if (pose.getY() < m_autoMinY) {
                m_autoMinY = pose.getY();
                m_autoMinYX = pose.getX();
                m_autoMinYT = t - 21;
            }
            m_autoMaxX = Math.max(m_autoMaxX, pose.getX());
        }
        if (m_step == 5 && !m_trenchReported && pose.getY() < 1.0 && pose.getX() > 3.6) {
            m_trenchReported = true;
            System.out.printf("SMOKE auto trench entry at t=%.1f: intake %.0f %% out%n", t - 21,
                100 * m_container.getIntakeRack().getExtensionFraction());
        }
        if (m_step == 5 && !m_autoStartReported && t > 21.1) {
            m_autoStartReported = true;
            System.out.println("SMOKE auto start pose" + (kRedAuto ? " (red, in the blue frame): " : ": ") + toBlueFrame(m_drive.getPose())
                + (kRedAuto ? "  field: " + m_drive.getPose() : ""));
        }
        if (m_step == 12 && !m_blockReported && t > 12.8) {
            m_blockReported = true;
            System.out.println("SMOKE intake while blocked: " + rack.status());
            for (var text : frc.robot.lib.util.AlertPublisher.active("Health", org.wpilib.util.Alert.Level.MEDIUM)) {
                System.out.println("SMOKE alert MEDIUM: " + text);
            }
        }
        if (m_step == 3 && !m_hitDone && t > 7.5) {
            m_hitDone = true;
            rack.simHitAndSkip(0.05, 0.03);
            System.out.println("SMOKE intake hit: left pushed in 5 cm, right skipped 3 cm: " + rack.status());
        }
        if ((m_step >= 1 && m_step <= 3) || m_step >= 10) {
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
        // Intake: first extend (finds the out stops), retract, second extend (Motion Magic only).
        var rack = m_container.getIntakeRack();
        if (step == 1) {
            m_pendingIntake = rack.extend();
        } else if (step == 2) {
            System.out.println("SMOKE intake after first extend: " + rack.status());
            m_pendingIntake = rack.retract();
        } else if (step == 3) {
            System.out.println("SMOKE intake after retract: " + rack.status());
            m_pendingIntake = rack.extend();
        } else if (step == 11) {
            System.out.printf("SMOKE intake after second extend and hit: %s; out in %.2f s%n", rack.status(), m_secondExtendTime);
            m_pendingIntake = rack.retract();
        } else if (step == 12) {
            m_pendingIntake = rack.extend();
        } else if (step == 13) {
            System.out.println("SMOKE intake after the blocked extend: " + rack.status());
            m_drive.resetPose(new Pose2d(2.5, 3.0, Rotation2d.ZERO));
            m_pendingIntake = m_container.getSuperstructure().shoot();
        } else if (step == 4) {
            var sup = m_container.getSuperstructure();
            var shot = sup.getShot();
            System.out.printf("SMOKE shoot: scoring %b, distance %.2f m, ready after %.2f s, fed %b, turret error %.2f deg,"
                    + " hood %.1f/%.1f deg, flywheel %.1f/%.1f rps%n",
                shot.scoring(), shot.distance(), m_readyTime, m_fed,
                (shot.turretRot() - m_container.getTurret().getPosition()) * 360,
                m_container.getHood().getAngleDeg(), shot.hoodDeg(), m_lastFlywheel, shot.flywheelRps());
            for (var level : org.wpilib.util.Alert.Level.values()) {
                for (var text : frc.robot.lib.util.AlertPublisher.active("Health", level)) {
                    System.out.println("SMOKE alert " + level + ": " + text);
                }
            }
            printFuel("teleop so far");
        }
        if (step == 11) {
            // Step 3 (stick released) just ended: the heading should not have moved.
            System.out.printf("SMOKE heading hold: %.1f deg at release, %.1f deg after 3 s%n",
                m_headingAtRelease, m_drive.getHeading().getDegrees());
        }
        if (step == 6) {
            System.out.println("SMOKE auto: " + m_container.getAutos().status()
                + (kRedAuto ? ", last target " + m_container.getSuperstructure().getShot().target()
                    + " (red hub " + FieldConstants.hub(true) + ")" : ""));
            System.out.printf("SMOKE auto bounds: lowest y %.2f m (wall 0) at x %.2f, t %.1f s; highest x %.2f m (center line 8.27)%n",
                m_autoMinY, m_autoMinYX, m_autoMinYT, m_autoMaxX);
            System.out.printf("SMOKE auto feed time %.1f s%n", m_autoFeedTime);
            System.out.printf("SMOKE auto intake leading while collecting: mean %.0f deg, max %.0f deg off the travel direction (%.1f s)%n",
                m_leadSum / m_leadTime, m_leadMax, m_leadTime);
            System.out.println("SMOKE auto intake leading worst at " + m_leadMaxAt);
            printFuel("auto");
            System.out.printf("SMOKE auto bump: highest z %.3f m, steepest pitch %.1f deg, odometry off the truth by up to %.2f m%n",
                m_autoMaxZ, m_autoMaxPitchDeg, m_autoMaxTruthError);
            m_drive.getSim().kickPose();
            report("after kick");
        }
    }

    private void printFuel(String when) {
        var fuel = m_container.getFuelSim().getField();
        System.out.printf("SMOKE fuel (%s): picked up %d, shots %d, scored blue %d red %d, hopper %d, balls on the field %d%n",
            when, fuel.pickedUp(), fuel.shots(), fuel.scored(false), fuel.scored(true), fuel.hopper(), fuel.ballCount());
    }

    /** Red poses turned 180 degrees about the field center, so they compare with the blue numbers. */
    private static Pose2d toBlueFrame(Pose2d pose) {
        if (!kRedAuto) {
            return pose;
        }
        return new Pose2d(FieldConstants.kFieldLength - pose.getX(), FieldConstants.kFieldWidth - pose.getY(),
            pose.getRotation().plus(Rotation2d.k180deg));
    }

    private static double parseStay() {
        try {
            return Double.parseDouble(System.getenv().getOrDefault("SIM_SMOKE_STAY", "0"));
        } catch (NumberFormatException e) {
            return 0;
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
