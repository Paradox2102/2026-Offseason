package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import java.text.DecimalFormat;
import java.util.function.Supplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Units;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * The swerve drive: CTRE's SwerveDrivetrain plus the commands and checks we add.
 *
 * <p>The CTRE drivetrain runs its own thread at 100 Hz (the most for classic CAN; CAN FD would
 * allow 250 Hz). That thread reads the motors, updates the pose and sends the last request to the
 * modules, so {@code setControl} only stores a request.
 *
 * <p>Two rules for every request in this code:
 * <ul>
 *   <li>Steer with {@code SteerRequestType.MotionMagicExpo}. Phoenix 26.70 defaults every request
 *       to {@code Position}, which gives the PID the whole angle change at once.
 *   <li>Drive with {@code DriveRequestType.Velocity} (closed loop). The motors then hold the asked
 *       speed whatever the battery voltage, and BLine's speed commands mean what they say.
 * </ul>
 */
public class Drive extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms: faster than odometry, so the sim PID behaves

    /** Blue drivers see forward as 0 degrees (toward the red wall); red drivers as 180 degrees. */
    private static final Rotation2d kBluePerspective = Rotation2d.ZERO;
    private static final Rotation2d kRedPerspective = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;
    private volatile boolean m_hasFieldPose = false; // set from the main loop and the sim thread

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private DriveSim m_sim = null;

    private final DriveTelemetry m_telemetry = new DriveTelemetry();

    /** Closed-loop robot-relative speeds, for BLine and the aim code. */
    private final SwerveRequest.ApplyRobotVelocity m_robotVelocity = new SwerveRequest.ApplyRobotVelocity()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    /** Which SysId test the Tuning buttons run. */
    public enum Routine { TRANSLATION, STEER, ROTATION }

    /*
     * Translation: the drive gains are in amps (TorqueCurrentFOC), so this test pushes with a
     * current, not a voltage. SysId only knows "volts": here "volts" means amps and "volts per
     * second" means amps per second. See SysIdSwerveTranslationTorque for the SysId tool setup.
     */
    private final SysIdSwerveTranslationTorque m_translationCharacterization = new SysIdSwerveTranslationTorque();
    private final SysIdRoutine m_sysIdTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(3).per(Second), // quasistatic: +3 A per second
            Volts.of(20), // dynamic: a 20 A step (about 3.5 m/s^2 for a 60 kg robot)
            null, // default timeout 10 s
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withAmps(output.in(Volts))), null, this));

    private final SysIdRoutine m_sysIdSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7),
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> setControl(new SwerveRequest.SysIdSwerveSteerGains().withVolts(volts)), null, this));

    /*
     * Rotation: finds the gains for the heading controller (DriveConstants.kHeadingP). SysId only
     * knows "volts", so here "volts" means rad/s and "volts per second" means rad/s^2.
     */
    private final SysIdRoutine m_sysIdRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second),
            Volts.of(Math.PI),
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(new SwerveRequest.SysIdSwerveRotation().withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this));

    /**
     * @param odometryHz odometry thread rate; 0 = the Phoenix default (250 Hz on CAN FD, 100 Hz on
     *     CAN 2.0; the SystemCore has CAN 2.0)
     * @param odometryStdDevs trust in wheel odometry, see {@link DriveConstants#kOdometryStdDevs}
     * @param visionStdDevs trust in a vision measurement sent without its own values
     */
    public Drive(SwerveDrivetrainConstants drivetrainConstants, double odometryHz, Matrix<N3, N1> odometryStdDevs,
                 Matrix<N3, N1> visionStdDevs, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryHz, odometryStdDevs, visionStdDevs, modules);
        if (Utils.isSimulation()) {
            m_sim = new DriveSim(this);
            startSimThread();
        }
    }

    @Override
    public void periodic() {
        /*
         * Set the driver's "forward" from the alliance. Before the first time, do it whatever the
         * mode, so a code restart in a match still drives the right way. After that, only while
         * disabled, so the controls never flip in the middle of driving.
         */
        if (!m_hasAppliedOperatorPerspective || RobotState.isDisabled()) {
            MatchState.getAlliance().ifPresent(alliance -> {
                setOperatorForwardDirection(alliance == Alliance.RED ? kRedPerspective : kBluePerspective);
                m_hasAppliedOperatorPerspective = true;
            });
        }
        m_telemetry.update(getState());
        if (m_sim != null) {
            m_sim.periodic();
        }
    }

    /** Field pose from odometry and vision, read live (a copy cached in periodic() is a loop old). */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /** Heading from the pose estimator. */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /** Measured speed in the robot's own frame (+x = robot front). */
    public ChassisVelocities getRobotVelocity() {
        return getState().Velocity;
    }

    /** Measured speed in the field frame (+x = toward the red wall). */
    public ChassisVelocities getFieldVelocity() {
        var state = getState();
        return state.Velocity.toFieldRelative(state.Pose.getRotation());
    }

    /**
     * True once the pose was set to a real field position: by vision's first pose (cameras that
     * agree, or a multi-tag frame; see Vision) or an auto start. Before that it is the field corner
     * (0, 0), and the gyro heading is only where the robot pointed at boot. A single vision
     * measurement blended into the corner pose does not count: it moves the pose only part of the way.
     */
    public boolean hasFieldPose() {
        return m_hasFieldPose;
    }

    /** Set the pose (auto start, driver button). In the sim, the simulated robot moves there too. */
    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
        m_hasFieldPose = true;
        if (m_sim != null) {
            m_sim.resetTruth(pose);
        }
    }

    /** Set only the estimate, not the simulated robot: the sim's "Kick Pose" test uses this. */
    void resetPoseEstimateOnly(Pose2d pose) {
        super.resetPose(pose);
    }

    /** Closed-loop drive at a robot-relative speed. BLine uses this. */
    public void driveRobotRelative(ChassisVelocities velocity) {
        /*
         * Discretize: a robot that drives and turns at the same time, with the same speeds held
         * for a whole loop, drifts sideways, because the "forward" direction turns during the loop.
         * discretize() corrects the speeds for that. FieldCentric does it inside Phoenix;
         * ApplyRobotVelocity does not, so we do it here (dt = one robot loop, 20 ms).
         */
        setControl(m_robotVelocity.withVelocity(velocity.discretize(TimedRobot.DEFAULT_PERIOD)));
    }

    /** Runs a request every loop while the command is scheduled. */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> setControl(request.get()));
    }

    /** Stop the wheels with a zero-speed request (SwerveRequest.Idle sends no command at all). */
    public void stop() {
        driveRobotRelative(new ChassisVelocities());
    }

    public static boolean isRedAlliance() {
        return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
    }

    /** Only in the simulator; null on the robot. */
    public DriveSim getSim() {
        return m_sim;
    }

    /** One SysId test. Run all four (quasistatic and dynamic, forward and reverse) per routine. */
    public Command sysId(Routine routine, SysIdRoutine.Direction direction, boolean dynamic) {
        SysIdRoutine r = switch (routine) {
            case TRANSLATION -> m_sysIdTranslation;
            case STEER -> m_sysIdSteer;
            case ROTATION -> m_sysIdRotation;
        };
        return (dynamic ? r.dynamic(direction) : r.quasistatic(direction))
            .withName("SysId " + routine + (dynamic ? " dynamic " : " quasistatic ") + direction);
    }

    /**
     * Measures the wheel radius: spin slowly in place, then compare how far the wheels rolled
     * with how far the gyro says the robot turned. Treads wear, so run it at each event. The
     * result prints when the command ends; put it in TunerConstants.kWheelRadius.
     */
    public Command wheelRadiusCharacterization() {
        SlewRateLimiter limiter = new SlewRateLimiter(DriveConstants.kWheelRadiusRampRate);
        // No deadband: the test spins at 0.5 rad/s.
        SwerveRequest.RobotCentric spin = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        double[] startPositions = new double[4];
        Rotation2d[] lastAngle = {Rotation2d.ZERO};
        double[] gyroDelta = {0};

        return Commands.parallel(
            Commands.sequence(
                Commands.runOnce(() -> limiter.reset(0.0)),
                run(() -> setControl(spin.withRotationalRate(limiter.calculate(DriveConstants.kWheelRadiusMaxVelocity))))),
            Commands.sequence(
                Commands.waitSeconds(1.0), // let the modules turn to the spin angles first
                Commands.runOnce(() -> {
                    System.arraycopy(wheelRadians(), 0, startPositions, 0, 4);
                    lastAngle[0] = getState().RawHeading; // raw gyro: vision must not count as turning
                    gyroDelta[0] = 0;
                }),
                Commands.run(() -> {
                    var heading = getState().RawHeading;
                    gyroDelta[0] += Math.abs(heading.minus(lastAngle[0]).getRadians());
                    lastAngle[0] = heading;
                }).finallyDo(() -> {
                    double[] end = wheelRadians();
                    double wheelDelta = 0;
                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(end[i] - startPositions[i]) / 4.0;
                    }
                    var f = new DecimalFormat("#0.0000");
                    if (gyroDelta[0] < Math.PI / 2 || wheelDelta < 1e-3) {
                        // Stopped in the first second, or turned too little to measure well.
                        System.out.println("Wheel radius test: too short (turned " + f.format(gyroDelta[0])
                            + " rad). Run it for at least one full turn.");
                        return;
                    }
                    double radius = gyroDelta[0] * DriveConstants.kDriveBaseRadius / wheelDelta;
                    System.out.println("********** Wheel Radius Characterization **********");
                    System.out.println("  Wheel delta: " + f.format(wheelDelta) + " rad");
                    System.out.println("  Gyro delta:  " + f.format(gyroDelta[0]) + " rad");
                    System.out.println("  Wheel radius: " + f.format(radius) + " m, "
                        + f.format(Units.metersToInches(radius)) + " in");
                })))
            .withName("Wheel Radius Characterization");
    }

    /** How far each wheel has turned, in radians (distance divided by the radius in use). */
    private double[] wheelRadians() {
        double[] values = new double[4];
        var positions = getState().ModulePositions;
        for (int i = 0; i < 4; i++) {
            values[i] = positions[i].distance / TunerConstants.FrontLeft.WheelRadius;
        }
        return values;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double now = Utils.getCurrentTimeSeconds();
            double dt = now - m_lastSimTime;
            m_lastSimTime = now;
            updateSimState(dt, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
