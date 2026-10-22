package frc.robot.health;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.system.RobotController;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import frc.robot.Robot;
import frc.robot.lib.util.AlertPublisher;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

/**
 * Live health checks for the swerve drive. HealthMonitor creates one and calls {@link #periodic()}
 * every loop.
 *
 * <p>The main idea: when the robot drives straight and steady, all four modules should do the SAME
 * thing. So each module is compared with the median of the four. A module that is different for
 * two windows in a row (2 s each) gets an alert that names it. The CAEL logs show the kinds of
 * problems this catches: the BL wheel was 7-23 % faster than the others with the lowest drive
 * current (slip or worn tread), and its steer error and steer current were about twice the others.
 *
 * <p>Every limit is a tunable under "Health/Swerve/...", so it can change without a new deploy.
 * Each window's ratios also go to the hoot log ("Health/Swerve/FL/SpeedRatio" and so on), so the
 * limits can be set from real match data.
 */
public final class SwerveHealth {
    private static final String k_group = "Health";
    private static final String[] k_corner = {"FL", "FR", "BL", "BR"}; // TunerConstants order
    /** 100 loops = 2 s of data per window. */
    private static final int k_windowSamples = 100;
    /** A module must be different in this many windows in a row before it gets an alert. */
    private static final int k_windowsToAlert = 2;
    private static final boolean k_showHardwareAlerts = Robot.isReal();

    // Limits, tunable live. The defaults come from the CAEL numbers.
    private final TunableDouble m_minSpeed = Tunables.addDouble("Health/Swerve/MinSpeedMps", 1.0);
    private final TunableDouble m_wheelSpeedLimit = Tunables.addDouble("Health/Swerve/WheelSpeedFraction", 0.07);
    private final TunableDouble m_clockingSpeedRatio = Tunables.addDouble("Health/Swerve/ClockingSpeedRatio", 0.70);
    private final TunableDouble m_driveCurrentLimit = Tunables.addDouble("Health/Swerve/DriveCurrentFraction", 0.30);
    private final TunableDouble m_steerErrorRatio = Tunables.addDouble("Health/Swerve/SteerErrorRatio", 1.8);
    private final TunableDouble m_steerCurrentRatio = Tunables.addDouble("Health/Swerve/SteerCurrentRatio", 1.4);
    private final TunableDouble m_steerAtLimitFraction = Tunables.addDouble("Health/Swerve/SteerAtLimitFraction", 0.20);

    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_drivetrain;
    private final int m_count;
    private final List<BaseStatusSignal> m_currentSignals = new ArrayList<>();
    private final BaseStatusSignal[] m_driveCurrent;
    private final BaseStatusSignal[] m_steerCurrent;
    private final double m_steerCurrentLimit;

    // Straight-driving window (checks 1, 2 and the live clocking check)
    private int m_straightSamples = 0;
    private final double[] m_speedSum;
    private final double[] m_driveCurrentSum;
    // Moving window (checks 3 and 4)
    private int m_movingSamples = 0;
    private final double[] m_steerErrorSum;
    private final double[] m_steerCurrentSum;
    private final int[] m_steerAtLimit;

    private final ModuleAlert[] m_clockingAlerts;
    private final ModuleAlert[] m_wheelSpeedAlerts;
    private final ModuleAlert[] m_driveCurrentAlerts;
    private final ModuleAlert[] m_steerErrorAlerts;
    private final ModuleAlert[] m_steerCurrentAlerts;

    // Gyro (check 8)
    private Rotation2d m_lastHeading = null;
    private double m_lastTimestamp = 0;
    private final Alert m_gyroJumpAlert = AlertPublisher.create(k_group, "swerve/gyroJump", "", Level.HIGH);
    private final Debouncer m_gyroDisagreeDebounce = new Debouncer(1.0, DebounceType.RISING);
    private final Alert m_gyroDisagreeAlert = AlertPublisher.create(k_group, "swerve/gyroDisagree", "", Level.LOW);

    // Odometry (check 7)
    private int m_failedDaqsAtWindowStart = -1;
    private double m_odometryWindowStart = 0;
    private double m_odometryAlertUntil = 0;
    private final Alert m_odometryAlert = AlertPublisher.create(k_group, "swerve/odometry", "", Level.MEDIUM);

    public SwerveHealth(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        m_drivetrain = drivetrain;
        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = drivetrain.getModules();
        m_count = modules.length;
        m_driveCurrent = new BaseStatusSignal[m_count];
        m_steerCurrent = new BaseStatusSignal[m_count];
        for (int i = 0; i < m_count; i++) {
            m_driveCurrent[i] = modules[i].getDriveMotor().getStatorCurrent(false);
            m_steerCurrent[i] = modules[i].getSteerMotor().getStatorCurrent(false);
            m_currentSignals.add(m_driveCurrent[i]);
            m_currentSignals.add(m_steerCurrent[i]);
        }
        m_steerCurrentLimit = TunerConstants.FrontLeft.SteerMotorInitialConfigs.CurrentLimits.StatorCurrentLimit;

        m_speedSum = new double[m_count];
        m_driveCurrentSum = new double[m_count];
        m_steerErrorSum = new double[m_count];
        m_steerCurrentSum = new double[m_count];
        m_steerAtLimit = new int[m_count];

        m_clockingAlerts = ModuleAlert.forEach(m_count, "clocking", Level.HIGH);
        m_wheelSpeedAlerts = ModuleAlert.forEach(m_count, "wheelSpeed", Level.MEDIUM);
        m_driveCurrentAlerts = ModuleAlert.forEach(m_count, "driveCurrent", Level.MEDIUM);
        m_steerErrorAlerts = ModuleAlert.forEach(m_count, "steerError", Level.MEDIUM);
        m_steerCurrentAlerts = ModuleAlert.forEach(m_count, "steerCurrent", Level.MEDIUM);
    }

    /** Call every loop (HealthMonitor does this). */
    public void periodic() {
        SwerveDriveState state = m_drivetrain.getStateCopy(); // a copy: the odometry thread writes the original
        if (state.ModuleVelocities == null || state.ModuleTargets == null || state.RawHeading == null) {
            return; // no data yet
        }
        BaseStatusSignal.refreshAll(m_currentSignals); // takes the latest values; it does not wait

        double gyroRate = checkGyro(state);
        checkOdometry(state);

        boolean usable = RobotState.isEnabled() && !RobotController.isBrownedOut();
        if (usable) {
            collectStraight(state, gyroRate);
            collectMoving(state);
        }
    }

    // ---------------------------------------------------------------------------------------
    // Checks 1, 2 and 6 (live): compare the modules while the robot drives straight and steady
    // ---------------------------------------------------------------------------------------

    private void collectStraight(SwerveDriveState state, double gyroRate) {
        // In pure translation every module gets the same target (speed and direction).
        // Add the target vectors to find the travel direction.
        double sumX = 0, sumY = 0;
        for (int i = 0; i < m_count; i++) {
            SwerveModuleVelocity t = state.ModuleTargets[i];
            sumX += t.velocity * t.angle.getCos();
            sumY += t.velocity * t.angle.getSin();
        }
        Translation2d travel = new Translation2d(sumX / m_count, sumY / m_count);
        double speed = travel.getNorm();
        if (speed < m_minSpeed.get() || Math.abs(gyroRate) > 0.3) {
            return;
        }
        for (int i = 0; i < m_count; i++) {
            SwerveModuleVelocity t = state.ModuleTargets[i];
            Translation2d target = new Translation2d(t.velocity * t.angle.getCos(), t.velocity * t.angle.getSin());
            if (target.getDistance(travel) > 0.1 * speed) {
                return; // not straight: one module has a different target
            }
        }

        // How fast does each wheel roll along the travel direction? A wheel that points the
        // wrong way (encoder clocked wrong) rolls at about cos(error) of the others.
        double[] along = new double[m_count];
        for (int i = 0; i < m_count; i++) {
            SwerveModuleVelocity m = state.ModuleVelocities[i];
            along[i] = (m.velocity * m.angle.getCos() * sumX + m.velocity * m.angle.getSin() * sumY)
                / (speed * m_count);
        }
        // The wheels must really move with the targets (not stale targets after Idle, not still
        // speeding up from a stop).
        if (median(along) < 0.5 * speed) {
            return;
        }
        for (int i = 0; i < m_count; i++) {
            m_speedSum[i] += along[i];
            m_driveCurrentSum[i] += Math.abs(m_driveCurrent[i].getValueAsDouble());
        }
        m_straightSamples++;
        if (m_straightSamples < k_windowSamples) {
            return;
        }

        double[] speedRatio = ratiosToMedian(m_speedSum);
        double[] currentRatio = ratiosToMedian(m_driveCurrentSum);
        double medianCurrent = median(m_driveCurrentSum) / m_straightSamples;
        for (int i = 0; i < m_count; i++) {
            String c = corner(i);
            SignalLogger.writeDouble("Health/Swerve/" + c + "/SpeedRatio", speedRatio[i]);
            SignalLogger.writeDouble("Health/Swerve/" + c + "/DriveCurrentRatio", currentRatio[i]);

            boolean clocking = speedRatio[i] < m_clockingSpeedRatio.get();
            m_clockingAlerts[i].update(clocking, String.format(
                "%s wheel at %.0f%% of the others' speed while driving straight: steer encoder probably clocked wrong (run Pit/Swerve Self-Test)",
                c, speedRatio[i] * 100));

            boolean wheelSpeed = !clocking && Math.abs(speedRatio[i] - 1) > m_wheelSpeedLimit.get();
            m_wheelSpeedAlerts[i].update(wheelSpeed, String.format(
                "%s wheel %.0f%% %s than the others while driving straight: check tread, slip and gearing",
                c, Math.abs(speedRatio[i] - 1) * 100, speedRatio[i] > 1 ? "faster" : "slower"));

            // A wrongly clocked wheel also fights the others and pulls more current. The clocking
            // alert already explains that, so do not add a second, misleading one.
            boolean driveCurrent = !clocking && medianCurrent > 3.0 && Math.abs(currentRatio[i] - 1) > m_driveCurrentLimit.get();
            m_driveCurrentAlerts[i].update(driveCurrent, String.format(
                "%s drive current %.0f%% %s than the others while driving straight: %s",
                c, Math.abs(currentRatio[i] - 1) * 100, currentRatio[i] > 1 ? "higher" : "lower",
                currentRatio[i] > 1 ? "check for drag or binding" : "the wheel may be slipping"));
        }
        m_straightSamples = 0;
        Arrays.fill(m_speedSum, 0);
        Arrays.fill(m_driveCurrentSum, 0);
    }

    // ---------------------------------------------------------------------------------------
    // Checks 3 and 4: steer tracking error and steer current while the robot moves
    // ---------------------------------------------------------------------------------------

    private void collectMoving(SwerveDriveState state) {
        double[] targetSpeeds = new double[m_count];
        double[] measuredSpeeds = new double[m_count];
        for (int i = 0; i < m_count; i++) {
            targetSpeeds[i] = Math.abs(state.ModuleTargets[i].velocity);
            measuredSpeeds[i] = Math.abs(state.ModuleVelocities[i].velocity);
        }
        // Both the targets AND the wheels must say "moving". Some requests (Idle) leave the last
        // targets in place after the robot stops.
        if (median(targetSpeeds) < 0.3 || median(measuredSpeeds) < 0.3) {
            return; // standing still: steer error has no meaning
        }
        for (int i = 0; i < m_count; i++) {
            // How far is the module from the angle it was told to hold?
            double error = Math.abs(state.ModuleTargets[i].angle.minus(state.ModuleVelocities[i].angle).getDegrees());
            if (error > 90) {
                error = 180 - error; // a module that points backwards and drives in reverse is not an error
            }
            m_steerErrorSum[i] += error;
            double current = Math.abs(m_steerCurrent[i].getValueAsDouble());
            m_steerCurrentSum[i] += current;
            if (m_steerCurrentLimit > 0 && current >= 0.95 * m_steerCurrentLimit) {
                m_steerAtLimit[i]++;
            }
        }
        m_movingSamples++;
        if (m_movingSamples < k_windowSamples) {
            return;
        }

        double medianError = median(m_steerErrorSum) / m_movingSamples;
        double[] currentRatio = ratiosToMedian(m_steerCurrentSum);
        for (int i = 0; i < m_count; i++) {
            String c = corner(i);
            double meanError = m_steerErrorSum[i] / m_movingSamples;
            double atLimit = (double) m_steerAtLimit[i] / m_movingSamples;
            double meanCurrent = m_steerCurrentSum[i] / m_movingSamples;
            SignalLogger.writeDouble("Health/Swerve/" + c + "/SteerErrorDeg", meanError, "deg");
            SignalLogger.writeDouble("Health/Swerve/" + c + "/SteerCurrentRatio", currentRatio[i]);
            SignalLogger.writeDouble("Health/Swerve/" + c + "/SteerAtLimitFraction", atLimit);

            boolean steerError = meanError > 5.0 && meanError > m_steerErrorRatio.get() * medianError;
            m_steerErrorAlerts[i].update(steerError, String.format(
                "%s steer error %.0f deg vs %.0f deg on the others: check the steer gears for binding or a loose pinion",
                c, meanError, medianError));

            boolean steerHot = (meanCurrent > 5.0 && currentRatio[i] > m_steerCurrentRatio.get())
                || atLimit > m_steerAtLimitFraction.get();
            m_steerCurrentAlerts[i].update(steerHot, String.format(
                "%s steer motor works too hard (%.1fx the others' current, at its limit %.0f%% of the time): check for binding",
                c, currentRatio[i], atLimit * 100));
        }
        m_movingSamples = 0;
        Arrays.fill(m_steerErrorSum, 0);
        Arrays.fill(m_steerCurrentSum, 0);
        Arrays.fill(m_steerAtLimit, 0);
    }

    // ---------------------------------------------------------------------------------------
    // Check 8: gyro heading jumps, and gyro vs wheels
    // ---------------------------------------------------------------------------------------

    /** @return the gyro turn rate in rad/s (0 when it cannot be measured) */
    private double checkGyro(SwerveDriveState state) {
        double rate = 0;
        double dt = state.Timestamp - m_lastTimestamp;
        if (m_lastHeading != null && dt > 0 && dt < 0.1) {
            double change = state.RawHeading.minus(m_lastHeading).getRadians();
            rate = change / dt;
            // The robot turns at most about 12 rad/s. A faster "turn" is a jump in the gyro
            // itself, for example after a Pigeon reboot. Then field-centric drive is rotated.
            if (Math.abs(rate) > 25) {
                m_gyroJumpAlert.setText(String.format(
                    "Gyro heading jumped %.0f deg at once (Pigeon reboot?): field-centric drive is now rotated, re-zero the heading",
                    Math.toDegrees(change)));
                m_gyroJumpAlert.set(k_showHardwareAlerts);
                SignalLogger.writeDouble("Health/Swerve/GyroJumpDeg", Math.toDegrees(change), "deg");
                rate = 0; // do not use a jump as a turn rate
            }
        }
        m_lastHeading = state.RawHeading;
        m_lastTimestamp = state.Timestamp;

        // The wheels also measure the turn rate (kinematics). A big difference for 1 s means
        // wheel slip, a push from another robot, or a gyro problem. Pushes are common, so LOW.
        double wheels = state.Velocity.omega;
        boolean disagree = RobotState.isEnabled()
            && Math.abs(rate - wheels) > Math.max(1.0, 0.5 * Math.abs(wheels));
        m_gyroDisagreeAlert.setText(String.format(
            "Gyro and wheels disagree on rotation (gyro %.1f, wheels %.1f rad/s): wheel slip, a push, or a gyro problem",
            rate, wheels));
        m_gyroDisagreeAlert.set(k_showHardwareAlerts && m_gyroDisagreeDebounce.calculate(disagree));
        return rate;
    }

    // ---------------------------------------------------------------------------------------
    // Check 7: the odometry thread could not read the module signals in time
    // ---------------------------------------------------------------------------------------

    private void checkOdometry(SwerveDriveState state) {
        SignalLogger.writeInteger("Health/Swerve/FailedDaqs", state.FailedDaqs);
        SignalLogger.writeInteger("Health/Swerve/SuccessfulDaqs", state.SuccessfulDaqs);
        SignalLogger.writeDouble("Health/Swerve/OdometryPeriod", state.OdometryPeriod * 1000.0, "ms");

        if (m_failedDaqsAtWindowStart < 0) {
            m_failedDaqsAtWindowStart = state.FailedDaqs;
            m_odometryWindowStart = state.Timestamp;
        }
        if (state.Timestamp - m_odometryWindowStart >= 5.0) {
            int failed = state.FailedDaqs - m_failedDaqsAtWindowStart;
            if (failed > 2) {
                m_odometryAlert.setText("Swerve odometry missed " + failed
                    + " reads in 5 s: CAN problem on the drive bus (check its wiring and the CAN alerts)");
                m_odometryAlertUntil = state.Timestamp + 10.0; // keep it visible for 10 s
            }
            m_failedDaqsAtWindowStart = state.FailedDaqs;
            m_odometryWindowStart = state.Timestamp;
        }
        m_odometryAlert.set(k_showHardwareAlerts && state.Timestamp < m_odometryAlertUntil);
    }

    // ---------------------------------------------------------------------------------------
    // Pit tools (check 6, parts 2 and 3)
    // ---------------------------------------------------------------------------------------

    /**
     * Pit self-test for steer encoder clocking. Run it after any work on a module.
     *
     * <p>The robot spins slowly in place for one turn. The gyro measures the true turn, so each
     * wheel must roll exactly (turn in radians) x (its distance from the center). A wheel that
     * points the wrong way rolls about cos(error) of that. The results print to the console, go to
     * the hoot log, and show as alerts.
     *
     * <p>Put the robot on carpet with about 1 m of space around it, and enable.
     */
    public static Command clockingSelfTest(Drive drive) {
        final double k_rate = 1.0; // rad/s, slow, so the test is safe in the pit
        SwerveRequest.RobotCentric spin = new SwerveRequest.RobotCentric(); // no deadband
        SelfTestState test = new SelfTestState(drive.getModules().length);

        return Commands.sequence(
            // 1 s for the modules to turn to the spin angles
            Commands.run(() -> drive.setControl(spin.withRotationalRate(k_rate)), drive).withTimeout(1.0),
            Commands.runOnce(() -> test.start(drive.getStateCopy())),
            Commands.run(() -> {
                drive.setControl(spin.withRotationalRate(k_rate));
                test.update(drive.getStateCopy());
            }, drive).until(() -> Math.abs(test.turned) >= 2 * Math.PI).withTimeout(15.0),
            Commands.runOnce(() -> test.finish(drive.getStateCopy(), drive.getModuleLocations()))
        ).finallyDo(() -> drive.setControl(new SwerveRequest.RobotCentric())) // zero speed: Idle sends no command at all
         .withName("Swerve Self-Test");
    }

    /** Point every wheel straight ahead, for a quick visual check that the modules are clocked right. */
    public static Command wheelsForward(Drive drive) {
        SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.ZERO);
        return Commands.run(() -> drive.setControl(point), drive)
            .finallyDo(() -> drive.setControl(new SwerveRequest.RobotCentric()))
            .withName("Wheels Forward");
    }

    /** The numbers that the self-test collects. */
    private static class SelfTestState {
        private static final Alert[] s_alerts = new Alert[4];
        private final double[] m_start;
        private final double[] m_sign;
        private Rotation2d m_lastHeading;
        double turned;

        SelfTestState(int count) {
            m_start = new double[count];
            m_sign = new double[count];
        }

        void start(SwerveDriveState state) {
            for (int i = 0; i < m_start.length; i++) {
                m_start[i] = state.ModulePositions[i].distance;
                // A module can point either way and drive backwards (module "optimization"), so
                // remember which way each one drives: the sign of its speed, flipped if the module
                // points opposite to its target angle.
                SwerveModuleVelocity target = state.ModuleTargets[i];
                boolean pointsOpposite = target.angle.minus(state.ModuleVelocities[i].angle).getCos() < 0;
                m_sign[i] = Math.signum(target.velocity) * (pointsOpposite ? -1 : 1);
            }
            m_lastHeading = state.RawHeading;
            turned = 0;
        }

        void update(SwerveDriveState state) {
            turned += state.RawHeading.minus(m_lastHeading).getRadians();
            m_lastHeading = state.RawHeading;
        }

        void finish(SwerveDriveState state, Translation2d[] locations) {
            update(state);
            int count = m_start.length;
            double[] ratio = new double[count];
            for (int i = 0; i < count; i++) {
                double rolled = (state.ModulePositions[i].distance - m_start[i]) * m_sign[i];
                double expected = Math.abs(turned) * locations[i].getNorm();
                ratio[i] = expected > 0 ? rolled / expected : 0;
            }
            // Compare with the median, so a wrong wheel radius (all four off the same amount)
            // does not look like a clocking problem.
            double median = median(ratio);
            System.out.println("********** Swerve Self-Test **********");
            System.out.println(String.format("\tTurned %.0f deg. All wheels together rolled %.0f%% of the expected distance%s",
                Math.toDegrees(turned), median * 100,
                Math.abs(median - 1) > 0.03 ? " (wheel radius may be off: run the wheel radius test)" : ""));
            for (int i = 0; i < count; i++) {
                double relative = median != 0 ? ratio[i] / median : 0;
                double errorDeg = Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, relative))));
                boolean bad = Math.abs(relative - 1) > 0.10;
                String c = corner(i);
                String line = String.format("%s rolled %.0f%% of the others (about %.0f deg off)%s",
                    c, relative * 100, errorDeg, bad ? ": steer encoder clocked wrong? Re-clock it, then run this test again" : ": OK");
                System.out.println("\t" + line);
                SignalLogger.writeDouble("Health/Swerve/SelfTest/" + c + "/Ratio", relative);
                if (i < s_alerts.length) {
                    if (s_alerts[i] == null) {
                        s_alerts[i] = AlertPublisher.create(k_group, "swerve/selfTest/" + i, "", Level.HIGH);
                    }
                    s_alerts[i].setText("Self-test: " + line);
                    s_alerts[i].set(bad);
                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------------

    /**
     * One alert per module. It turns on after the problem shows in {@link #k_windowsToAlert}
     * windows in a row, and then stays on until the code restarts (worn tread does not heal).
     */
    private static class ModuleAlert {
        private final Alert m_alert;
        private int m_windowsInARow = 0;

        ModuleAlert(String id, Level level) {
            m_alert = AlertPublisher.create(k_group, id, "", level);
        }

        static ModuleAlert[] forEach(int count, String check, Level level) {
            ModuleAlert[] alerts = new ModuleAlert[count];
            for (int i = 0; i < count; i++) {
                alerts[i] = new ModuleAlert("swerve/" + check + "/" + i, level);
            }
            return alerts;
        }

        void update(boolean problem, String text) {
            m_windowsInARow = problem ? m_windowsInARow + 1 : 0;
            if (m_windowsInARow >= k_windowsToAlert) {
                m_alert.setText(text);
                m_alert.set(k_showHardwareAlerts);
            }
        }
    }

    private static String corner(int i) {
        return i < k_corner.length ? k_corner[i] : "Module " + i;
    }

    private static double median(double[] values) {
        double[] sorted = values.clone();
        Arrays.sort(sorted);
        int n = sorted.length;
        return n % 2 == 1 ? sorted[n / 2] : (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;
    }

    private static double[] ratiosToMedian(double[] values) {
        double median = median(values);
        double[] ratios = new double[values.length];
        for (int i = 0; i < values.length; i++) {
            ratios[i] = median != 0 ? values[i] / median : 1.0;
        }
        return ratios;
    }
}
