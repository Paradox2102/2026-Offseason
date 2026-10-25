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
 * runs the trench auto, kicks the pose to see vision pull it back, prints a summary
 * ("SMOKE ..." lines) and exits. Any exception in the robot code shows in the output, which the
 * script checks.
 */
public class SimSmokeTest {
    /** SIM_SMOKE_ALLIANCE=red: run the auto as red (teleop and the shooting step stay blue). */
    private static final boolean kRedAuto = "red".equalsIgnoreCase(System.getenv("SIM_SMOKE_ALLIANCE"));
    /**
     * SIM_SMOKE_AUTO=right|left|right-sotm|left-sotm: the trench auto to run (sotm = the double
     * trench run that shoots on the move). A left auto's checks are mirrored onto the right side.
     */
    private static final String kAuto = System.getenv().getOrDefault("SIM_SMOKE_AUTO", "right").toLowerCase();
    private static final boolean kLeftAuto = kAuto.startsWith("left");
    private static final boolean kSotmAuto = kAuto.endsWith("sotm");
    private static final String kAutoName = (kLeftAuto ? "Left" : "Right")
        + (kSotmAuto ? " trench x2, shoot on the move" : " trench + bump x2");
    /**
     * SIM_SMOKE_RACK: a rack fault for the whole auto.
     * <ul>
     *   <li>blocked: something blocks the left side at 0.10 m. The intake never gets fully out:
     *       the auto must not go under the trench (fallback path), and no shot goes.
     *   <li>short: something holds the right side 2.4 cm short of its out stop (the frame flexes,
     *       the left side gets out). Same result: the short side must not count as out.
     *   <li>dead: Talon 22 stops answering before auto. Fallback path, and the shot goes anyway.
     * </ul>
     */
    private static final String kRackFault = System.getenv().getOrDefault("SIM_SMOKE_RACK", "").toLowerCase();
    /** SIM_SMOKE_STAY=<seconds>: keep the sim running after the script, to look at it in AdvantageScope. */
    private static final double kStaySeconds = parseStay();
    /** The script ends here; then it stays up kStaySeconds. */
    private static final double kScriptEnd = 57;
    private final RobotContainer m_container;
    private final Drive m_drive;
    // Scripted sticks. (DriverStationSim joystick axes did not reach Gamepad in a headless run,
    // so the teleop command gets these values directly; the GUI sim with a real gamepad works.)
    private double m_leftY = 0;
    private double m_leftX = 0;
    private double m_rightX = 0;
    private final Timer m_timer = new Timer();
    private int m_step = -1;
    private int m_headingAtKick = 0;
    private int m_multiTagAtKick = 0;
    private double m_maxTeleopSpeed = 0;
    private double m_headingAtRelease = 0;
    private boolean m_teleopScheduled = false;
    private org.wpilib.command2.Command m_pendingIntake = null;
    private org.wpilib.command2.Command m_shotCommand = null;
    private org.wpilib.command2.Command m_snap = null;
    private double m_headingBeforeSnap = Double.NaN;
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
    /** Feed time while the robot moved faster than 0.5 m/s, and the speed then (shoot on the move). */
    private double m_autoFeedMovingTime = 0;
    private double m_autoFeedMovingSpeedSum = 0;
    private int m_autoCrossingsSeen = 0;
    private int m_autoShotsSeen = 0;
    /** Auto: when the last ball was shot and the last one scored, seconds from the auto start. */
    private double m_autoLastShot = Double.NaN;
    private double m_autoLastScored = Double.NaN;
    private double m_leadSum = 0;
    private double m_leadTime = 0;
    private double m_leadMax = 0;
    private String m_leadMaxAt = "";
    private double m_readyTime = Double.NaN;
    private boolean m_fed = false;
    private double m_firstFeedIntakeOut = Double.NaN;
    private boolean m_firstFeedIntakeExtended = false;
    private double m_lastFlywheel = 0;
    /** The teleop shot: first feed, the pack and the clear after the release. */
    private double m_firstFeedTime = Double.NaN;
    private double m_packStartTime = Double.NaN;
    private double m_packMin = Double.POSITIVE_INFINITY;
    private double m_packMax = Double.NEGATIVE_INFINITY;
    private double m_packFeedTime = 0;
    private double m_packTime = 0;
    private double m_runOnTime = 0;
    private double m_clearStart = Double.NaN;
    private double m_clearTime = 0;
    private double m_clearRollerSpeed = 0;
    private double m_autoMaxZ = 0;
    private double m_autoMaxPitchDeg = 0;
    private double m_autoMaxTruthError = 0;
    private int m_autoContactsAtStart = -1;
    private boolean m_trenchRunScheduled = false;
    private double m_autoMaxPartX = Double.NEGATIVE_INFINITY;
    private double m_autoMinPartXMax = Double.NEGATIVE_INFINITY;
    private double m_autoMaxPartY = Double.NEGATIVE_INFINITY;
    private String m_autoMaxPartXAt = "";
    private boolean m_autoWasInZone = true;
    private double m_autoHeadingAt65 = Double.NaN;
    private double m_autoHeadingAt70 = Double.NaN;
    private double m_autoTurnDone = Double.NaN;
    private String m_autoTurnDoneAt = "never";
    private double m_autoSweepHeadingErr = 0;
    private double m_autoStopTime = Double.NaN;
    private final java.util.List<String> m_autoReturns = new java.util.ArrayList<>();
    private String m_autoMaxPartYAt = "";
    private double m_hoodBeforeTrench = Double.NaN;
    private double m_rackBeforeTrench = Double.NaN;
    private int m_teleopContactsAtStart = -1;
    private double m_teleopMinHeadroom = Double.POSITIVE_INFINITY;
    private double m_teleopMinRackUnder = Double.POSITIVE_INFINITY;
    private double m_teleopMaxHoodUnder = 0;
    private double m_teleopGuardTime = 0;
    private boolean m_teleopFedNearTrench = false;
    private double m_autoMinHeadroom = Double.POSITIVE_INFINITY;
    private boolean m_firstPoseReported = false;
    private double m_firstPoseError = Double.NaN;
    private double m_firstPoseHeadingError = Double.NaN;

    public SimSmokeTest(RobotContainer container, Drive drive) {
        m_container = container;
        m_drive = drive;
        m_container.selectAuto(kAutoName);
        // Start in the blue zone, where the cameras can see tags, with NO field pose: the robot is
        // there, the drive does not know it (pose at the corner, gyro 67 degrees off the field, as
        // on the practice field 2026-10-25). Vision must find the first pose (checked before step 2).
        m_drive.getSim().placeTruthOnly(new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(-67)));
        // The sim trusts multi-tag headings; the robot does not yet (kCamerasChecked false), so
        // there only cameras that agree can give the first pose. Test that path.
        m_container.getVision().setMultiTagSeedForTest(false);
        DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        m_timer.start();
    }

    public void periodic() {
        double t = m_timer.get();
        watchFirstPose(t);
        if (t < 1) {
            step(0, "disabled", RobotMode.TELEOPERATED, false);
        } else if (t < 3) {
            // The robot stays in the blue zone here (user: no drive over the bump into the fuel and
            // across the center line before the auto). The teleop trench run at the end drives.
            step(1, "teleop: stand still, intake first extend", RobotMode.TELEOPERATED, true);
            m_leftY = 0;
            m_leftX = 0;
            m_rightX = 0;
        } else if (t < 4) {
            step(10, "teleop: stand still", RobotMode.TELEOPERATED, true);
        } else if (t < 6) {
            step(2, "teleop: turn in place", RobotMode.TELEOPERATED, true);
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
        } else if (t < 19.2) {
            step(13, "teleop: shoot from (2.5, 3.0) in the blue zone", RobotMode.TELEOPERATED, true);
        } else if (t < 20) {
            // A match starts with the intake in: the auto must put it out before the trench.
            step(15, "teleop: shot stopped, intake in", RobotMode.TELEOPERATED, true);
        } else if (t < 21) {
            step(4, kRedAuto ? "disabled, alliance set to red" : "disabled", RobotMode.TELEOPERATED, false);
            if (kRedAuto) {
                DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
            }
        } else if (t < 41) {
            step(5, "auto: " + kAutoName + " (20 s)", RobotMode.AUTONOMOUS, true);
        } else if (t < 42) {
            step(6, "disabled, kick pose", RobotMode.AUTONOMOUS, false);
        } else if (t < 49) {
            step(7, "disabled, vision corrects", RobotMode.AUTONOMOUS, false);
        } else if (t < 50) {
            step(8, "disabled, robot to (2.0, 0.65) in the blue zone, lined up with the right trench", RobotMode.TELEOPERATED, false);
            m_leftY = 0;
            m_leftX = 0;
            m_rightX = 0;
        } else if (t < 51) {
            step(9, "teleop: intake in, shot prepared (hood up)", RobotMode.TELEOPERATED, true);
        } else if (t < kScriptEnd) {
            step(14, "teleop: drive through the trench with the intake in and the hood up", RobotMode.TELEOPERATED, true);
            m_leftY = -0.45;
            trackTeleopTrench();
        } else if (m_step != 99) {
            step(99, "script done", RobotMode.TELEOPERATED, false);
            report("end");
            writeTopicList();
            if (t >= kScriptEnd + kStaySeconds) {
                System.exit(0);
            }
            System.out.println("SMOKE staying up for " + kStaySeconds + " s (SIM_SMOKE_STAY) for AdvantageScope");
        } else if (t >= kScriptEnd + kStaySeconds) {
            System.exit(0);
        }
        // The scheduler drops a command scheduled while disabled, so wait for the enable.
        if (m_step == 9 && !m_trenchRunScheduled && RobotState.isEnabled()) {
            // The auto took the drive; teleop drive again, the intake in, the shot prepared.
            CommandScheduler.getInstance().schedule(DriveCommands.teleop(
                m_drive, () -> m_leftY, () -> m_leftX, () -> m_rightX, () -> false));
            CommandScheduler.getInstance().schedule(m_container.getIntakeRack().retract());
            CommandScheduler.getInstance().schedule(m_container.getSuperstructure().prepare());
            m_trenchRunScheduled = true;
        }
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
            boolean feeding = m_container.getFeeder().isFeeding();
            if (feeding && !m_fed) {
                m_firstFeedIntakeOut = m_container.getIntakeRack().getPosition();
                m_firstFeedIntakeExtended = m_container.getIntakeRack().isExtended();
            }
            m_fed |= feeding;
            m_lastFlywheel = m_container.getFlywheel().getSpeed();
            if (feeding && Double.isNaN(m_firstFeedTime)) {
                m_firstFeedTime = t;
            }
            if (rack.isPacking()) {
                if (Double.isNaN(m_packStartTime)) {
                    m_packStartTime = t;
                }
                m_packTime += 0.020;
                if ("1".equals(System.getenv("SIM_SMOKE_TRACE"))) {
                    System.out.printf("SMOKE pack trace t %.2f: rack %.3f m, feeding %b%n", t - m_packStartTime, rack.getPosition(), feeding);
                }
                m_packFeedTime += feeding ? 0.020 : 0;
                // After it got there: the pulse around the pack position.
                if (t - m_packStartTime > 0.3) {
                    m_packMin = Math.min(m_packMin, rack.getPosition());
                    m_packMax = Math.max(m_packMax, rack.getPosition());
                }
            }
        }
        if (m_step == 15) {
            // The shot was released at t 19.2: the vertical roller runs on, then everything runs back.
            var feeder = m_container.getFeeder();
            if (feeder.isClearing()) {
                if (Double.isNaN(m_clearStart)) {
                    m_clearStart = t - 19.2;
                }
                m_clearTime += 0.020;
                m_clearRollerSpeed = Math.min(m_clearRollerSpeed, feeder.verticalRollerSpeed());
            } else if (Double.isNaN(m_clearStart) && feeder.verticalRollerSpeed() > 5) {
                m_runOnTime += 0.020;
            }
        }
        // The rack must be out (lower) before the robot reaches the trench. The auto checks use the
        // blue frame: a red pose is turned 180 degrees about the field center first.
        var pose = toBlueFrame(m_drive.getPose());
        // From the start pose on (the first loop of the auto still has the teleop pose).
        if (m_step == 5 && m_autoStartReported) {
            var crossings = m_container.getFuelSim().getField().crossings();
            for (int i = m_autoCrossingsSeen; i < crossings.size(); i++) {
                var c = crossings.get(i);
                if (c.scored()) {
                    m_autoLastScored = t - 21;
                }
                if (!c.scored() || "1".equals(System.getenv("SIM_SMOKE_TRACE"))) {
                    System.out.printf("SMOKE auto " + (c.scored() ? "hit" : "miss") + " at t %.1f: %.2f m from the hub center (shot %.1f s before, robot now at %s)%n",
                        t - 21, c.miss(), c.flightTime(), toBlueFrame(m_drive.getPose()).getTranslation());
                }
            }
            m_autoCrossingsSeen = crossings.size();
            int shots = m_container.getFuelSim().getField().shots();
            if (shots > m_autoShotsSeen) {
                m_autoLastShot = t - 21;
            }
            if ("1".equals(System.getenv("SIM_SMOKE_TRACE")) && shots > m_autoShotsSeen) {
                var sup = m_container.getSuperstructure();
                var shot = sup.getShot();
                var fv = m_drive.getFieldVelocity();
                System.out.printf("SMOKE shot %d at t %.2f: %s %.1f m/s %.1f rad/s, d %.2f, turret %.1f deg off, hood %.1f off,"
                        + " wheel %.1f %% off%n", shots, t - 21, toBlueFrame(m_drive.getPose()).getTranslation(), Math.hypot(fv.vx, fv.vy),
                    m_drive.getRobotVelocity().omega, shot.distance(),
                    (m_container.getTurret().getPosition() - shot.turretAimRot()) * 360,
                    m_container.getHood().getAngleDeg() - shot.hoodDeg(),
                    (m_container.getFlywheel().getSpeed() / shot.flywheelRps() - 1) * 100);
            }
            m_autoShotsSeen = shots;
            if ("1".equals(System.getenv("SIM_SMOKE_TRACE")) && Math.round((t - 21) * 50) % 5 == 0) {
                var fv = m_drive.getFieldVelocity();
                var sup = m_container.getSuperstructure();
                System.out.printf("SMOKE trace t %.1f: %s %.1f m/s, hopper %d, feeding %b, blocked by '%s', aim d %.2f,"
                        + " rack %.3f m%s%s%n", t - 21,
                    toBlueFrame(m_drive.getPose()).getTranslation(), Math.hypot(fv.vx, fv.vy),
                    m_container.getFuelSim().getField().hopper(), m_container.getFeeder().isFeeding(), sup.blockedBy(),
                    sup.getShot().distance(), m_container.getIntakeRack().getPosition(),
                    m_container.getIntakeRack().isPacking() ? " packing" : "", sup.isNearTrench() ? " near trench" : "");
            }
            if (m_container.getFeeder().isFeeding()) {
                m_autoFeedTime += 0.020;
                var fv = m_drive.getFieldVelocity();
                double speed = Math.hypot(fv.vx, fv.vy);
                if (speed > 0.5) {
                    m_autoFeedMovingTime += 0.020;
                    m_autoFeedMovingSpeedSum += speed * 0.020;
                }
            }
            // After the trench: how soon the robot faces the center line (90 degrees, intake leading
            // up the sweep). Only on the way out (before the first return).
            if (m_autoReturns.isEmpty()) {
                double heading = pose.getRotation().getDegrees();
                if (Double.isNaN(m_autoHeadingAt65) && pose.getX() > 6.5) {
                    m_autoHeadingAt65 = heading;
                }
                if (Double.isNaN(m_autoHeadingAt70) && pose.getX() > 7.0) {
                    m_autoHeadingAt70 = heading;
                }
                if (Double.isNaN(m_autoTurnDone) && pose.getX() > 5.3 && Math.abs(heading - 90) < 15) {
                    m_autoTurnDone = t - 21;
                    m_autoTurnDoneAt = String.format("(%.2f, %.2f) at t %.1f s", pose.getX(), pose.getY(), t - 21);
                }
                // Overshoot or wobble once turned, up the sweep.
                if (!Double.isNaN(m_autoTurnDone) && pose.getX() > 7.3 && pose.getY() < 2.9) {
                    m_autoSweepHeadingErr = Math.max(m_autoSweepHeadingErr, Math.abs(heading - 90));
                }
            }
            // Back into our zone: is the turret already on the hub, and how long after the robot
            // stops does the first ball go?
            var sup = m_container.getSuperstructure();
            boolean inZone = pose.getX() < FieldConstants.k_blueZoneTransition;
            if (inZone && !m_autoWasInZone) {
                m_autoReturns.add(String.format("entered the zone at t %.1f s, turret %.0f deg off the hub", t - 21,
                    (sup.getShot().turretAimRot() - m_container.getTurret().getPosition()) * 360));
                m_autoStopTime = Double.NaN;
            }
            m_autoWasInZone = inZone;
            double speedNow = Math.hypot(m_drive.getFieldVelocity().vx, m_drive.getFieldVelocity().vy);
            if (inZone && Double.isNaN(m_autoStopTime) && speedNow < 0.15 && !m_autoReturns.isEmpty()) {
                m_autoStopTime = t;
            }
            if (inZone && !Double.isNaN(m_autoStopTime) && m_container.getFeeder().isFeeding()
                    && !m_autoReturns.isEmpty() && !m_autoReturns.get(m_autoReturns.size() - 1).contains("first ball")) {
                int last = m_autoReturns.size() - 1;
                m_autoReturns.set(last, m_autoReturns.get(last) + String.format(", stopped at t %.1f s, first ball %.2f s after the stop",
                    m_autoStopTime - 21, t - m_autoStopTime));
            }
            // Over the bump: how high and how steep, and how far odometry runs from the truth.
            var truth3d = m_drive.getSim().truthPose3d();
            m_autoMaxZ = Math.max(m_autoMaxZ, truth3d.getZ());
            m_autoMaxPitchDeg = Math.max(m_autoMaxPitchDeg, Math.toDegrees(Math.abs(truth3d.getRotation().getY())));
            m_autoMaxTruthError = Math.max(m_autoMaxTruthError,
                m_drive.getPose().getTranslation().getDistance(m_drive.getSim().truthPose().getTranslation()));
            // Under the trench: how much room is left above the robot (the intake must be out).
            var sim = m_drive.getSim();
            if (m_autoContactsAtStart < 0) {
                m_autoContactsAtStart = sim.contacts();
            }
            // Center lines, in the blue frame: never fully across x 8.27 (all of the robot past the
            // tape's far edge, 8.296); stay below y 4.03 (the tape starts at 3.984).
            double minX = Double.POSITIVE_INFINITY;
            var blueTruth = toBlueFrame(sim.truthPose());
            String at = String.format("robot (%.2f, %.2f) heading %.0f, t %.1f s", blueTruth.getX(), blueTruth.getY(),
                blueTruth.getRotation().getDegrees(), t - 21);
            for (var c : frc.robot.sim.field.RobotOutline.corners(blueTruth, sim.rackOut())) {
                if (c.getX() > m_autoMaxPartX) {
                    m_autoMaxPartX = c.getX();
                    m_autoMaxPartXAt = at;
                }
                if (c.getY() > m_autoMaxPartY) {
                    m_autoMaxPartY = c.getY();
                    m_autoMaxPartYAt = at;
                }
                minX = Math.min(minX, c.getX());
            }
            m_autoMinPartXMax = Math.max(m_autoMinPartXMax, minX);
            m_autoMinHeadroom = Math.min(m_autoMinHeadroom, frc.robot.sim.field.FieldObstacles.trenchHeadroom(
                sim.truthPose(), sim.rackOut(), IntakeConstants.kRackStroke, sim.hoodDeg(), truth3d.getZ()));
            // Intake leading: in the neutral zone, while moving, how far the robot front (the
            // intake) points from the direction of travel.
            var v = m_drive.getFieldVelocity();
            double speed = Math.hypot(v.vx, v.vy);
            double vxBlue = kRedAuto ? -v.vx : v.vx;
            double vyBlue = (kRedAuto ? -v.vy : v.vy) * (kLeftAuto ? -1 : 1);
            // Only the collecting part: after the turn at the trench exit (it backs out of the
            // trench, intake trailing, on purpose) and along the center line, not the drive back.
            if (pose.getX() > 6.4 && speed > 0.5 && vxBlue > -0.5 * speed) {
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
        if (m_step == 5 && !m_trenchReported && m_autoStartReported && pose.getY() < 1.0 && pose.getX() > 3.95) {
            m_trenchReported = true;
            System.out.printf("SMOKE auto trench entry at t=%.1f: intake %.0f %% out%n", t - 21,
                100 * m_container.getIntakeRack().getExtensionFraction());
        }
        if (m_step == 5 && !m_autoStartReported && t > 21.1) {
            m_autoStartReported = true;
            System.out.println("SMOKE auto start pose" + (kLeftAuto ? " (left auto, mirrored onto the right side)" : "")
                + (kRedAuto ? " (red, in the blue frame): " : ": ") + toBlueFrame(m_drive.getPose())
                + (kRedAuto ? "  field: " + m_drive.getPose() : ""));
            // A robot must start with some part over its starting line (blue frame x 3.978-4.029).
            double lo = Double.POSITIVE_INFINITY;
            double hi = Double.NEGATIVE_INFINITY;
            for (var c : frc.robot.sim.field.RobotOutline.corners(toBlueFrame(m_drive.getSim().truthPose()), 0)) {
                lo = Math.min(lo, c.getX());
                hi = Math.max(hi, c.getX());
            }
            System.out.printf("SMOKE auto start: robot spans x %.3f-%.3f, starting line 3.978-4.029: overlaps %b%n",
                lo, hi, hi >= 3.978 && lo <= 4.029);
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

    /** When the drive first has a field pose: how (vision seed or multi-tag), when, how good. */
    private void watchFirstPose(double t) {
        if (m_firstPoseReported || !m_drive.hasFieldPose()) {
            return;
        }
        m_firstPoseReported = true;
        var vision = m_container.getVision();
        Pose2d pose = m_drive.getPose();
        Pose2d truth = m_drive.getSim().truthPose();
        m_firstPoseError = pose.getTranslation().getDistance(truth.getTranslation());
        m_firstPoseHeadingError = Math.abs(pose.getRotation().minus(truth.getRotation()).getDegrees());
        System.out.printf("SMOKE first field pose at t %.2f s: %d camera seeds, %d multi-tag seeds; pose (%.2f, %.2f, %.1f deg)"
                + " truth (%.2f, %.2f, %.1f deg), error %.3f m, heading %.1f deg%n",
            t, vision.seeds(), vision.seedsFromMultiTag(), pose.getX(), pose.getY(), pose.getRotation().getDegrees(),
            truth.getX(), truth.getY(), truth.getRotation().getDegrees(), m_firstPoseError, m_firstPoseHeadingError);
    }

    /** Before the robot moves: vision must have found the first field pose, near the truth. */
    private void checkFirstPose() {
        if (!m_firstPoseReported) {
            System.out.println("SMOKE FAIL: no field pose before the robot moves (vision did not seed it)");
        } else if (m_firstPoseError > 0.3 || m_firstPoseHeadingError > 5) {
            System.out.printf("SMOKE FAIL: first field pose off by %.3f m, %.1f deg%n", m_firstPoseError, m_firstPoseHeadingError);
        }
    }

    private void step(int step, String name, RobotMode mode, boolean enabled) {
        if (step == m_step) {
            return;
        }
        if (step == 2) {
            checkFirstPose();
        }
        if (m_step >= 0) {
            report("end of step " + m_step);
        }
        if (m_step == 7) {
            checkVisionCorrected();
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
            // Hold X (snap to the nearest 90 degrees) while the robot stands still.
            m_headingBeforeSnap = m_drive.getHeading().getDegrees();
            m_snap = DriveCommands.snapToNearestQuarterTurn(m_drive, () -> 0.0, () -> 0.0);
            CommandScheduler.getInstance().schedule(m_snap);
        } else if (step == 12) {
            CommandScheduler.getInstance().cancel(m_snap);
            System.out.printf("SMOKE snap to nearest 90 (X): heading %.1f deg before, target %.0f deg, %.1f deg after 2 s%n",
                m_headingBeforeSnap, DriveCommands.nearestQuarterTurn(Rotation2d.fromDegrees(m_headingBeforeSnap)).getDegrees(),
                m_drive.getHeading().getDegrees());
            m_pendingIntake = rack.extend();
        } else if (step == 13) {
            System.out.println("SMOKE intake after the blocked extend: " + rack.status());
            m_drive.resetPose(new Pose2d(2.5, 3.0, Rotation2d.ZERO));
            // Intake in first: the shot must put it out, and feed only once it is fully out.
            m_shotCommand = org.wpilib.command2.Commands.sequence(rack.retract(), m_container.getSuperstructure().shoot());
            m_pendingIntake = m_shotCommand;
        } else if (step == 15) {
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
            System.out.printf("SMOKE shoot with the intake in first: at the first feed the intake was %.3f m out, fully out %b%n",
                m_firstFeedIntakeOut, m_firstFeedIntakeExtended);
            System.out.printf("SMOKE shot pack: first feed at %.2f s, pack %.2f s after it, packing %.2f s (fed %.2f s of it),"
                    + " rack %.3f-%.3f m while pulsing (pack position %.3f m, out stop %.3f)%n",
                m_firstFeedTime - 16, m_packStartTime - m_firstFeedTime, m_packTime, m_packFeedTime, m_packMin, m_packMax,
                0.85 * IntakeConstants.kOutStopLeft, IntakeConstants.kOutStopLeft);
            printFuel("teleop so far");
            CommandScheduler.getInstance().cancel(m_shotCommand);
            m_pendingIntake = rack.retract();
        } else if (step == 4) {
            if (kRackFault.equals("dead")) {
                rack.simKillRight();
                System.out.println("SMOKE auto: Talon 22 (right) stops answering (SIM_SMOKE_RACK=dead)");
            }
        } else if (step == 5) {
            System.out.println("SMOKE intake at the start of auto: " + rack.status() + ", fully out " + rack.isExtended());
            if (kRackFault.equals("blocked")) {
                rack.simBlockLeft(0.10);
                System.out.println("SMOKE auto: left rack side blocked at 0.10 m (SIM_SMOKE_RACK=blocked)");
            } else if (kRackFault.equals("short")) {
                rack.simBlockRight(IntakeConstants.kOutStopRight - 0.024);
                System.out.println("SMOKE auto: right rack side held 2.4 cm short of its out stop (SIM_SMOKE_RACK=short)");
            }
        }
        if (step == 4) {
            System.out.printf("SMOKE shot release: vertical roller ran on %.2f s, then every stage ran back from %.2f s"
                    + " for %.2f s (vertical roller down to %.0f rps); intake %s%n",
                m_runOnTime, m_clearStart, m_clearTime, m_clearRollerSpeed, rack.status());
        }
        if (step == 11) {
            // Step 3 (stick released) just ended: the heading should not have moved.
            System.out.printf("SMOKE heading hold: %.1f deg at release, %.1f deg after 3 s%n",
                m_headingAtRelease, m_drive.getHeading().getDegrees());
        }
        if (step == 8) {
            DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
            m_drive.resetPose(new Pose2d(2.0, 0.65, Rotation2d.ZERO));
        }
        if (step == 99) {
            var sim = m_drive.getSim();
            System.out.printf("SMOKE teleop trench: before it, intake %.3f m out and hood %.1f deg; near the trench %.1f s;"
                    + " under the arm: intake at least %.3f m out (stroke %.3f), hood at most %.1f deg, headroom %.3f m;"
                    + " fed near the trench %b; field contacts %d (last: %s); robot reached x %.2f%n",
                m_rackBeforeTrench, m_hoodBeforeTrench, m_teleopGuardTime, m_teleopMinRackUnder, IntakeConstants.kRackStroke,
                m_teleopMaxHoodUnder, m_teleopMinHeadroom, m_teleopFedNearTrench, sim.contacts() - m_teleopContactsAtStart,
                sim.lastContact(), sim.truthPose().getX());
        }
        if (step == 6) {
            m_container.getIntakeRack().simBlockLeft(Double.NaN);
            m_container.getIntakeRack().simBlockRight(Double.NaN);
            System.out.println("SMOKE auto intake at the end: " + m_container.getIntakeRack().status()
                + ", fully out " + m_container.getIntakeRack().isExtended());
            for (var text : frc.robot.lib.util.AlertPublisher.active("Health", org.wpilib.util.Alert.Level.HIGH)) {
                System.out.println("SMOKE auto alert HIGH: " + text);
            }
            System.out.println("SMOKE feeder: recent jams " + m_container.getFeeder().recentJams()
                + ", hood " + String.format("%.1f", m_container.getHood().getAngleDeg()) + " deg");
            System.out.println("SMOKE auto: " + m_container.getAutos().status()
                + (kRedAuto ? ", last target " + m_container.getSuperstructure().getShot().target()
                    + " (red hub " + FieldConstants.hub(true) + ")" : ""));
            System.out.printf("SMOKE auto bounds: lowest y %.2f m (wall 0) at x %.2f, t %.1f s; highest x %.2f m (center line 8.27)%n",
                m_autoMinY, m_autoMinYX, m_autoMinYT, m_autoMaxX);
            System.out.printf("SMOKE auto feed time %.1f s, %.1f s of it moving (mean %.1f m/s)%n", m_autoFeedTime,
                m_autoFeedMovingTime, m_autoFeedMovingTime == 0 ? 0 : m_autoFeedMovingSpeedSum / m_autoFeedMovingTime);
            System.out.printf("SMOKE auto intake leading while collecting: mean %.0f deg, max %.0f deg off the travel direction (%.1f s)%n",
                m_leadSum / m_leadTime, m_leadMax, m_leadTime);
            System.out.println("SMOKE auto intake leading worst at " + m_leadMaxAt);
            printFuel("auto");
            System.out.printf("SMOKE auto finish (auto ends at 20.0 s): last ball shot at %.1f s, last ball scored at %.1f s,"
                    + " %d balls left in the hopper%n", m_autoLastShot, m_autoLastScored,
                m_container.getFuelSim().getField().hopper());
            System.out.printf("SMOKE auto turn after the trench: heading %.0f deg at x 6.5, %.0f deg at x 7.0; within 15 deg of 90 at %s"
                    + " (fuel starts at x 7.43); up the sweep at most %.0f deg off 90%n", m_autoHeadingAt65, m_autoHeadingAt70,
                m_autoTurnDoneAt, m_autoSweepHeadingErr);
            for (int i = 0; i < m_autoReturns.size(); i++) {
                System.out.println("SMOKE auto return " + (i + 1) + ": " + m_autoReturns.get(i));
            }
            System.out.printf("SMOKE auto bump: highest z %.3f m, steepest pitch %.1f deg, odometry off the truth by up to %.2f m%n",
                m_autoMaxZ, m_autoMaxPitchDeg, m_autoMaxTruthError);
            System.out.printf("SMOKE auto center lines: farthest part at x %.2f, nearest part at most x %.2f"
                    + " (fully across = more than 8.296); highest part y %.2f (long center line tape from 3.984)%n",
                m_autoMaxPartX, m_autoMinPartXMax, m_autoMaxPartY);
            System.out.println("SMOKE auto center lines: farthest x at " + m_autoMaxPartXAt + "; highest y at " + m_autoMaxPartYAt);
            System.out.printf("SMOKE auto field contacts: %d loops touching a structure (last: %s); trench headroom %.3f m%n",
                m_drive.getSim().contacts() - m_autoContactsAtStart, m_drive.getSim().lastContact(), m_autoMinHeadroom);
            m_drive.getSim().kickPose();
            m_headingAtKick = m_container.getVision().headingMeasurements();
            m_multiTagAtKick = m_container.getVision().multiTagMeasurements();
            report("after kick");
        }
    }

    /** The teleop trench run: the guard must put the intake out and the hood down in time. */
    private void trackTeleopTrench() {
        var sim = m_drive.getSim();
        var sup = m_container.getSuperstructure();
        if (m_teleopContactsAtStart < 0) {
            m_teleopContactsAtStart = sim.contacts();
            m_hoodBeforeTrench = m_container.getHood().getAngleDeg();
            m_rackBeforeTrench = m_container.getIntakeRack().getPosition();
        }
        if (sup.isNearTrench()) {
            m_teleopGuardTime += 0.020;
            m_teleopFedNearTrench |= m_container.getFeeder().isFeeding();
        }
        double headroom = frc.robot.sim.field.FieldObstacles.trenchHeadroom(sim.truthPose(), sim.rackOut(),
            IntakeConstants.kRackStroke, sim.hoodDeg(), sim.truthPose3d().getZ());
        if (Double.isFinite(headroom)) {
            m_teleopMinHeadroom = Math.min(m_teleopMinHeadroom, headroom);
            m_teleopMinRackUnder = Math.min(m_teleopMinRackUnder, sim.rackOut());
            m_teleopMaxHoodUnder = Math.max(m_teleopMaxHoodUnder, sim.hoodDeg());
        }
    }

    private void printFuel(String when) {
        var fuel = m_container.getFuelSim().getField();
        System.out.printf("SMOKE fuel (%s): picked up %d, shots %d, scored blue %d red %d, hopper %d, balls on the field %d%n",
            when, fuel.pickedUp(), fuel.shots(), fuel.scored(false), fuel.scored(true), fuel.hopper(), fuel.ballCount());
    }

    /** Red poses turned 180 degrees about the field center, so they compare with the blue numbers. */
    private static Pose2d toBlueFrame(Pose2d pose) {
        Pose2d blue = !kRedAuto ? pose : new Pose2d(FieldConstants.kFieldLength - pose.getX(),
            FieldConstants.kFieldWidth - pose.getY(), pose.getRotation().plus(Rotation2d.k180deg));
        // The left auto mirrored about the long center line, onto the right auto's side (the
        // same mirror tools/mirror-paths.py makes), so every check reads the same.
        return !kLeftAuto ? blue : new Pose2d(blue.getX(), FieldConstants.kFieldWidth - blue.getY(), blue.getRotation().unaryMinus());
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

    /**
     * End of step 7: vision must have pulled the kicked pose (0.5 m and 10 degrees off) back to the
     * truth, the heading too. tools/sim-smoke.sh fails on the "SMOKE FAIL" line.
     */
    private void checkVisionCorrected() {
        var pose = m_drive.getPose();
        var truth = m_drive.getSim().truthPose();
        double error = pose.getTranslation().getDistance(truth.getTranslation());
        double headingError = Math.abs(truth.getRotation().minus(pose.getRotation()).getDegrees());
        // Counted from the kick: frames from the auto before it say nothing about the correction.
        int heading = m_container.getVision().headingMeasurements() - m_headingAtKick;
        int multiTag = m_container.getVision().multiTagMeasurements() - m_multiTagAtKick;
        System.out.printf("SMOKE vision after the kick: %d multi-tag measurements, %d with a heading%n",
            multiTag, heading);
        if (error > 0.05 || headingError > 1.0) {
            System.out.printf("SMOKE FAIL: vision did not correct the kick: error %.3f m, heading %.1f deg"
                + " (%d multi-tag, %d with a heading since the kick)%n", error, headingError, multiTag, heading);
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
