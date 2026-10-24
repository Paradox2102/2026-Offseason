package frc.robot.sim;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.system.Timer;

import frc.robot.RobotContainer;
import frc.robot.calibration.Calibration;
import frc.robot.calibration.CalibrationStore;
import frc.robot.calibration.FeederProcedure;
import frc.robot.calibration.HoodZeroProcedure;
import frc.robot.calibration.IntakeZeroProcedure;
import frc.robot.calibration.ShotTableProcedure;
import frc.robot.calibration.SteerZeroProcedure;
import frc.robot.calibration.TurretZeroProcedure;
import frc.robot.generated.TunerConstants;
import frc.robot.health.SwerveHealth;
import frc.robot.subsystems.drive.Drive;

/**
 * Simulator only: the calibration procedures, for {@code tools/sim-calibration.sh}. Runs only when
 * the environment variable SIM_CALIB=1, with its own calibration file (CALIBRATION_FILE).
 *
 * <p>In Utility mode it runs each procedure through the same methods the test gamepad calls,
 * checks that a recorded shot point changes the shot the robot uses, saves, and reads the file
 * back. "CALIB ..." lines; a failed check prints "CALIB FAIL".
 *
 * <p>SIM_CALIB_BOOT=1: a second run, booted with the file the first one saved: each value in it
 * must be in use from boot on.
 */
public class SimCalibrationTest {
    private final RobotContainer m_container;
    private final Drive m_drive;
    private final Calibration m_calibration;
    private final Timer m_timer = new Timer();
    private int m_step = -1;
    private Command m_command = null;
    private double m_distance = Double.NaN;
    private double m_tableBefore = Double.NaN;
    private double m_farBefore = Double.NaN;
    private int m_scoredBefore = 0;
    private int m_shotsBefore = 0;
    private double m_stepStart = 0;
    private boolean m_midDone = false;
    /** Next loop: the shot the robot works out now uses the recorded point. */
    private boolean m_liveCheck = false;
    private Command m_hoodJog = null;
    /** Turret angle when the hold check starts, degrees. */
    private double m_turretBeforeDeg = Double.NaN;

    public SimCalibrationTest(RobotContainer container, Drive drive) {
        m_container = container;
        m_drive = drive;
        m_calibration = container.getCalibration();
        DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
        DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
        DriverStationSim.setEnabled(false);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        m_drive.resetPose(new Pose2d(2.5, 3.0, Rotation2d.ZERO));
        m_timer.start();
    }

    /** Booted with the saved file: every value in it is in use. */
    private void bootCheck() {
        var data = m_calibration.data();
        System.out.println("CALIB boot with the saved file");
        var point = data.shotPoints.get(0);
        double[] shot = m_container.getSuperstructure().shotAt(point.distance());
        check(Math.abs(shot[0] - point.flywheelRps()) < 1e-6 && Math.abs(shot[1] - point.hoodDeg()) < 1e-6,
            String.format("shot point in the table: %.1f rps, %.1f deg at %.2f m", shot[0], shot[1], point.distance()));
        check(java.util.Arrays.equals(TunerConstants.encoderOffsets(), data.steerOffsets),
            "steer offsets in use: " + java.util.Arrays.toString(TunerConstants.encoderOffsets()));
        check(Math.abs(m_container.getTurret().getTrim() * 360 - data.turretTrimDeg) < 1e-9,
            String.format("turret trim %.2f deg", m_container.getTurret().getTrim() * 360));
        var rack = m_container.getIntakeRack();
        check(rack.outStop(true) == data.intakeOutStopLeft && rack.outStop(false) == data.intakeOutStopRight,
            String.format("intake out stops %.4f, %.4f m", rack.outStop(true), rack.outStop(false)));
        check(m_container.getHood().ratio() == data.hoodRatio, String.format("hood ratio %.2f", m_container.getHood().ratio()));
        check(java.util.Arrays.equals(m_container.getFeeder().feedSpeeds(), data.feedSpeeds),
            "feed speeds " + java.util.Arrays.toString(m_container.getFeeder().feedSpeeds()));
    }

    private void check(boolean ok, String what) {
        System.out.println("CALIB " + (ok ? "ok: " : "FAIL: ") + what);
    }

    public void periodic() {
        double t = m_timer.get();
        if ("1".equals(System.getenv("SIM_CALIB_BOOT"))) {
            if (t > 1) {
                bootCheck();
                System.out.println("CALIB end");
                System.exit(0);
            }
            return;
        }
        // The loop after the record: the superstructure has worked out the shot with it.
        if (m_liveCheck) {
            m_liveCheck = false;
            var point = m_calibration.data().shotPoints.get(0);
            var live = m_container.getSuperstructure().getShot();
            check(Math.abs(live.distance() - point.distance()) > 0.05 || Math.abs(live.flywheelRps() - point.flywheelRps()) < 0.5,
                String.format("the robot's shot now: %.1f rps at %.3f m (recorded %.1f rps at %.3f m)", live.flywheelRps(),
                    live.distance(), point.flywheelRps(), point.distance()));
        }
        if (t < 2) {
            step(0, RobotMode.TELEOPERATED, false);
        } else if (t < 3) {
            step(1, RobotMode.UTILITY, true);
        } else if (t < 8) {
            step(2, RobotMode.UTILITY, true); // shot table: shoot with jogged values
        } else if (t < 9) {
            step(3, RobotMode.UTILITY, true); // record, check, save
        } else if (t < 11) {
            step(4, RobotMode.UTILITY, true); // steer: wheels forward, capture
        } else if (t < 14) {
            step(5, RobotMode.UTILITY, true); // turret trim
        } else if (t < 26) {
            step(6, RobotMode.UTILITY, true); // intake zero
        } else if (t < 33) {
            step(7, RobotMode.UTILITY, true); // hood zero
        } else if (t < 37) {
            step(8, RobotMode.UTILITY, true); // feeder
        } else if (t < 38) {
            step(9, RobotMode.UTILITY, false); // hood zero picked while disabled; the robot moves
        } else if (t < 40) {
            step(10, RobotMode.UTILITY, true); // enabled: the turret holds still
        } else if (t < 41) {
            step(11, RobotMode.UTILITY, false); // turret zero picked while disabled
        } else if (t < 45) {
            step(12, RobotMode.UTILITY, true); // enabled: jig zero, then the 90 deg check turn
        } else if (m_step != 99) {
            step(99, RobotMode.UTILITY, false);
            System.out.println("CALIB end");
            System.exit(0);
        }
        if (!m_midDone) {
            m_midDone = midStep(m_step, t - m_stepStart);
        }
        // Commands scheduled while disabled are dropped: start them once enabled.
        if (m_command != null && org.wpilib.driverstation.RobotState.isEnabled() && !m_command.isScheduled()
                && t - m_stepStart > 0.1) {
            CommandScheduler.getInstance().schedule(m_command);
            m_command = null;
        }
        DriverStationSim.notifyNewData();
    }

    private void step(int step, RobotMode mode, boolean enabled) {
        if (step == m_step) {
            return;
        }
        endStep(m_step);
        m_step = step;
        m_stepStart = m_timer.get();
        m_midDone = false;
        DriverStationSim.setRobotMode(mode);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
        var sup = m_container.getSuperstructure();
        switch (step) {
            case 0 -> {
                System.out.println("CALIB step 0: teleop, disabled: the tool must do nothing");
                check(!m_calibration.save(), "save refused outside Utility mode");
            }
            case 1 -> System.out.println("CALIB step 1: Utility mode, enabled; robot at (2.5, 3.0)");
            case 2 -> {
                m_calibration.select("Shot table");
                var shot = m_calibration.procedure(ShotTableProcedure.class);
                m_distance = shot.distance();
                m_tableBefore = sup.getShot().flywheelRps();
                m_farBefore = sup.shotAt(5.5)[0];
                shot.fromTable();
                shot.jogFlywheel(5);
                shot.jogFlywheel(1);
                shot.jogHood(2);
                m_scoredBefore = m_container.getFuelSim().getField().scored(false);
                m_shotsBefore = m_container.getFuelSim().getField().shots();
                System.out.printf("CALIB step 2: shot table at %.2f m: table %.1f rps; shooting with %.1f rps, hood %.1f deg%n",
                    m_distance, m_tableBefore, shot.flywheelRps(), shot.hoodDeg());
                m_command = shot.shoot();
            }
            case 4 -> {
                m_calibration.select("Steer zero");
                m_command = SwerveHealth.wheelsForward(m_drive);
                System.out.println("CALIB step 4: steer zero: wheels forward, then capture");
            }
            case 5 -> {
                m_calibration.select("Turret zero");
                System.out.println("CALIB step 5: turret zero: turn to 3 deg, then record the trim");
            }
            case 6 -> {
                m_calibration.select("Intake zero");
                m_calibration.procedure(IntakeZeroProcedure.class).capture();
                System.out.println("CALIB step 6: intake zero: three times out and in");
            }
            case 7 -> {
                m_calibration.select("Hood zero");
                var hood = m_calibration.procedure(HoodZeroProcedure.class);
                hood.home();
                hood.setTyped(42, 18);
                m_hoodJog = m_container.getHood().jogUp();
                m_command = m_hoodJog;
                System.out.println("CALIB step 7: hood zero: home, up to the upper stop (42 deg in the sim), ratio");
            }
            case 8 -> {
                m_calibration.select("Feeder tuning");
                var feeder = m_calibration.procedure(FeederProcedure.class);
                double before = m_container.getFeeder().feedSpeeds()[0];
                feeder.jog(5);
                check(Math.abs(m_container.getFeeder().feedSpeeds()[0] - before - 5) < 1e-9,
                    String.format("indexer feed speed %.1f -> %.1f rps, used at once", before, m_container.getFeeder().feedSpeeds()[0]));
                m_command = feeder.fireRateTest();
                System.out.println("CALIB step 8: feeder: fire-rate test (the sim flywheel has no ball load: no dips expected)");
            }
            case 9 -> {
                // Picked while disabled, as at the robot. The robot moves, so aiming would turn the turret.
                m_calibration.select("Hood zero");
                m_drive.resetPose(new Pose2d(5.0, 6.5, Rotation2d.ZERO));
                System.out.println("CALIB step 9: Utility mode disabled, hood zero picked, robot moved to (5.0, 6.5)");
            }
            case 10 -> {
                m_turretBeforeDeg = m_container.getTurret().getPosition() * 360;
                System.out.printf("CALIB step 10: enabled: the turret must hold %.1f deg, not aim at the hub%n", m_turretBeforeDeg);
            }
            case 11 -> {
                m_calibration.select("Turret zero");
                System.out.println("CALIB step 11: Utility mode disabled, turret zero picked");
            }
            case 12 -> System.out.println("CALIB step 12: enabled: jig zero, then the turret turns 90 deg clockwise");
            default -> {}
        }
    }

    /**
     * Things to do part way through a step.
     *
     * @return true when done for this step
     */
    private boolean midStep(int step, double sinceStart) {
        switch (step) {
            case 5 -> {
                // The procedure's hold starts at the turret's angle; then turn it to 3 deg.
                if (sinceStart < 0.3) {
                    return false;
                }
                var turret = m_calibration.procedure(TurretZeroProcedure.class);
                turret.jog(3.0 - m_container.getTurret().getPosition() * 360);
                return true;
            }
            case 12 -> {
                // The hold starts at the enable; then the jig zero (the sim turret is not in a jig:
                // where it is becomes 0).
                if (sinceStart < 0.3) {
                    return false;
                }
                m_calibration.procedure(TurretZeroProcedure.class).capture();
                return true;
            }
            case 7 -> {
                // Capture while the jog holds the hood on its upper stop.
                if (sinceStart < 5.0) {
                    return false;
                }
                System.out.printf("CALIB hood at the upper stop: %.1f deg, %.2f motor turns from the lower stop%n",
                    m_container.getHood().getAngleDeg(), m_container.getHood().motorTurnsFromStop());
                m_calibration.procedure(HoodZeroProcedure.class).capture();
                CommandScheduler.getInstance().cancel(m_hoodJog);
                return true;
            }
            default -> {
                return true;
            }
        }
    }

    private void endStep(int step) {
        var sup = m_container.getSuperstructure();
        switch (step) {
            case 2 -> {
                CommandScheduler.getInstance().cancelAll();
                int scored = m_container.getFuelSim().getField().scored(false) - m_scoredBefore;
                int shots = m_container.getFuelSim().getField().shots() - m_shotsBefore;
                System.out.printf("CALIB shot with the jogged values: %d balls shot, %d scored%n", shots, scored);
                var shot = m_calibration.procedure(ShotTableProcedure.class);
                double rps = shot.flywheelRps();
                double hood = shot.hoodDeg();
                shot.record();
                double at = m_calibration.data().shotPoints.get(0).distance();
                check(Math.abs(sup.shotAt(at)[0] - rps) < 1e-6 && Math.abs(sup.shotAt(at)[1] - hood) < 1e-6,
                    String.format("recorded point used at %.3f m: table %.1f -> %.1f rps, hood %.1f deg", at, m_tableBefore,
                        sup.shotAt(at)[0], sup.shotAt(at)[1]));
                m_liveCheck = true;
                check(Math.abs(sup.shotAt(5.5)[0] - m_farBefore) < 1e-6, "the shot at 5.5 m is unchanged");
            }
            case 3 -> {
                // The jogs stay for a small move and go for a move past 0.1 m.
                var shot = m_calibration.procedure(ShotTableProcedure.class);
                shot.jogFlywheel(3);
                double jogged = shot.flywheelRps();
                shot.setTypedDistance(m_distance + 0.05);
                check(Math.abs(shot.flywheelRps() - jogged) < 1e-9,
                    String.format("moved 0.05 m: jogged %.1f rps kept", jogged));
                double far = m_distance + 1.0;
                shot.setTypedDistance(far);
                double[] table = sup.shotAt(far);
                check(Math.abs(shot.flywheelRps() - table[0]) < 1e-9 && Math.abs(shot.hoodDeg() - table[1]) < 1e-9,
                    String.format("moved 1 m: table values loaded at %.2f m (%.1f rps, %.1f deg)", far, table[0], table[1]));
                shot.setTypedDistance(0);
                var point = m_calibration.data().shotPoints.get(0);
                check(Math.abs(shot.flywheelRps() - point.flywheelRps()) < 0.5,
                    String.format("back at the pose (%.2f m): %.1f rps, the recorded point's %.1f", shot.distance(),
                        shot.flywheelRps(), point.flywheelRps()));
            }
            case 4 -> {
                CommandScheduler.getInstance().cancelAll();
                var steer = m_calibration.procedure(SteerZeroProcedure.class);
                steer.capture();
                double[] inUse = TunerConstants.encoderOffsets();
                double[] captured = steer.captured();
                double worst = 0;
                for (int i = 0; i < inUse.length; i++) {
                    worst = Math.max(worst, Math.abs(captured[i] - inUse[i]) * 360);
                }
                System.out.printf("CALIB steer: in use %s, captured %s%n", java.util.Arrays.toString(inUse), java.util.Arrays.toString(captured));
                check(worst < 2, String.format("wheels forward in the sim: new offsets within %.2f deg of those in use", worst));
            }
            case 5 -> {
                var turret = m_calibration.procedure(TurretZeroProcedure.class);
                turret.recordTrim();
                double trim = m_container.getTurret().getTrim() * 360;
                check(Math.abs(trim - 3.0) < 0.5, String.format("turret trim %.2f deg (jogged 3)", trim));
                check(Math.abs(m_container.getTurret().getPosition() * 360) < 0.5,
                    String.format("turret reads %.2f deg with the trim", m_container.getTurret().getPosition() * 360));
            }
            case 6 -> {
                var rack = m_container.getIntakeRack();
                check(!m_calibration.procedure(IntakeZeroProcedure.class).isRunning(), "intake zero finished in 12 s");
                System.out.printf("CALIB intake out stops: left %.4f m, right %.4f m (sim stops: 0.324, 0.330)%n",
                    rack.outStop(true), rack.outStop(false));
                check(Math.abs(rack.outStop(true) - 0.324) < 0.004 && Math.abs(rack.outStop(false) - 0.330) < 0.004,
                    "each side measured its own out stop");
            }
            case 7 -> {
                double ratio = m_container.getHood().ratio();
                check(Math.abs(ratio - 85) < 85 * 0.03, String.format("hood ratio %.2f (the sim's is 85)", ratio));
            }
            case 10 -> {
                double now = m_container.getTurret().getPosition() * 360;
                double aimDeg = m_container.getSuperstructure().getShot().turretRot() * 360;
                check(Math.abs(now - m_turretBeforeDeg) < 1.0 && Math.abs(aimDeg - m_turretBeforeDeg) > 10,
                    String.format("turret held %.1f deg (was %.1f; aiming would be %.1f)", now, m_turretBeforeDeg, aimDeg));
            }
            case 12 -> {
                var turret = m_container.getTurret();
                double now = turret.getPosition() * 360;
                check(turret.isZeroed() && Math.abs(now + 90) < 1.0,
                    String.format("after the jig zero the turret turned to %.1f deg (check: -90, clockwise)", now));
            }
            case 8 -> {
                CommandScheduler.getInstance().cancelAll();
                System.out.printf("CALIB feeder fire-rate test: %d dips%n", m_calibration.procedure(FeederProcedure.class).dips());
                check(m_calibration.save(), "saved");
                try {
                    var path = CalibrationStore.defaultPath();
                    var back = new CalibrationStore(path).load();
                    var data = m_calibration.data();
                    check(back.shotPoints.equals(data.shotPoints) && back.hoodRatio == data.hoodRatio
                        && back.intakeOutStopRight == data.intakeOutStopRight
                        && java.util.Arrays.equals(back.feedSpeeds, data.feedSpeeds)
                        && java.util.Arrays.equals(back.steerOffsets, data.steerOffsets),
                        "the file reads back the same (" + path + ")");
                    System.out.print(java.nio.file.Files.readString(path));
                } catch (java.io.IOException e) {
                    check(false, "read back: " + e);
                }
            }
            default -> {}
        }
    }
}
