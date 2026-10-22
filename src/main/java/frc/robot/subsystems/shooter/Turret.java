package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.constants.CanIds;
import frc.robot.health.HealthMonitor;
import frc.robot.lib.phoenix.PhoenixUtil;
import frc.robot.lib.util.AlertPublisher;

/**
 * The turret (Talon 44). Angles are turret rotations; 0 = the jig position (shooter to the
 * robot's left). Positive = counterclockwise seen from above.
 *
 * <p>The turret has no absolute sensor and no limit switch: only the motor's own encoder, which
 * counts from where the motor was at power-on. So the turret must be in its jig when the robot is
 * turned on. If the Talon reboots in a match, it counts from wherever the turret is then, so
 * every angle after that would be wrong: the turret stops and an alert asks for the jig and the
 * "Zero turret (in jig)" button.
 */
public class Turret extends SubsystemBase {
    private final TalonFX m_talon = new TalonFX(CanIds.shooterAzimuth, CanIds.k_canBus3);
    private final MotionMagicExpoTorqueCurrentFOC m_request = new MotionMagicExpoTorqueCurrentFOC(0);
    private final NeutralOut m_neutral = new NeutralOut();
    private final Alert m_lostZeroAlert = AlertPublisher.create("Health", "shooter/turret/zero",
        "Turret Talon 44 rebooted: its zero is lost. Turret stopped: put it in the jig and press Pit > Zero turret", Level.HIGH);
    private final double m_startTime = Timer.getTimestamp();
    private boolean m_zeroed = true; // in the jig at power-on
    private double m_goal = 0;
    private static final double kSimPeriod = 0.005;
    private DCMotorSim m_sim = null;
    private Notifier m_simNotifier = null;

    public Turret() {
        var config = new TalonFXConfiguration();
        // Check on the robot: a positive command must turn the turret counterclockwise (seen from
        // above). If it turns the other way, change this to Clockwise_Positive.
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.kTurretRatio;
        config.Slot0.kS = ShooterConstants.kTurretS;
        config.Slot0.kP = ShooterConstants.kTurretP;
        config.Slot0.kD = ShooterConstants.kTurretD;
        config.MotionMagic.MotionMagicExpo_kV = ShooterConstants.kTurretExpoV;
        config.MotionMagic.MotionMagicExpo_kA = ShooterConstants.kTurretExpoA;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants.kTurretMaxRot;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants.kTurretMinRot;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kTurretStatorLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        PhoenixUtil.applyConfig("Turret (44)", () -> m_talon.getConfigurator().apply(config));
        // No setPosition(0) here. A Talon reads 0 when it powers on, and the turret is in its jig
        // at power-on, so 0 = jig. A robot code restart (a redeploy, a crash) does not restart
        // the Talon: it still knows the true angle, and zeroing here would make wherever the
        // turret points the new 0 (and move the cable limits with it).
        m_talon.hasResetOccurred(); // clear the flag of this boot
        HealthMonitor.register(m_talon);

        if (RobotBase.isSimulation()) {
            var motor = DCMotor.getKrakenX60Foc(1);
            m_sim = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(motor, 0.0497, ShooterConstants.kTurretRatio), motor);
            var state = m_talon.getSimState();
            state.Orientation = ChassisReference.CounterClockwise_Positive;
            state.setMotorType(TalonFXSimState.MotorType.KrakenX60);
            m_simNotifier = new Notifier(this::simStep);
            m_simNotifier.startPeriodic(kSimPeriod);
        }
    }

    @Override
    public void periodic() {
        boolean reset = m_talon.hasResetOccurred();
        if (reset && Timer.getTimestamp() - m_startTime > 3.0) {
            m_zeroed = false;
            m_lostZeroAlert.set(true);
        }
        if (!RobotState.isEnabled()) {
            // Disabled: the Talon is neutral anyway; send nothing (at code start it may not be up).
        } else if (m_zeroed) {
            m_talon.setControl(m_request.withPosition(m_goal));
        } else {
            m_talon.setControl(m_neutral);
        }
        Telemetry.log("Shooter/Turret/Goal deg", m_goal * 360);
        Telemetry.log("Shooter/Turret/Position deg", getPosition() * 360);
        Telemetry.log("Shooter/Turret/Error deg", (m_goal - getPosition()) * 360);
        Telemetry.log("Shooter/Turret/Zeroed", m_zeroed);
    }

    /** Turret angle, rotations from the jig. */
    public double getPosition() {
        return m_talon.getPosition().getValueAsDouble();
    }

    public boolean isZeroed() {
        return m_zeroed;
    }

    /** Zeroed and within the tolerance of {@code goal}. */
    public boolean isOnTarget(double goal) {
        return isOnTarget(goal, ShooterConstants.kTurretToleranceDeg);
    }

    /** Zeroed, on CAN (a lost Talon's last reading stays frozen), and within {@code toleranceDeg}. */
    public boolean isOnTarget(double goal, double toleranceDeg) {
        return m_zeroed && m_talon.isConnected() && Math.abs(getPosition() - goal) * 360 < toleranceDeg;
    }

    /** Point the turret at the angle from the supplier, every loop, until interrupted. */
    public Command track(DoubleSupplier goal) {
        return run(() -> m_goal = goal.getAsDouble()).withName("Turret Track");
    }

    /** The turret is in its jig now: this is 0 again. */
    public Command zeroInJig() {
        return Commands.runOnce(() -> {
            m_talon.setPosition(0);
            m_goal = 0;
            m_zeroed = true;
            m_lostZeroAlert.set(false);
        }).ignoringDisable(true).withName("Zero turret (in jig)");
    }

    /**
     * Sim physics every 5 ms, not once per 20 ms loop: the Talon runs its loop at 1 kHz, and a
     * turret position up to 20 ms old made the stiff loop swing in the sim (not on the robot,
     * where the Talon reads its own encoder).
     */
    private void simStep() {
        var state = m_talon.getSimState();
        state.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_sim.setInputVoltage(state.getMotorVoltage());
        m_sim.update(kSimPeriod);
        double turretRot = m_sim.getAngularPosition() / (2 * Math.PI);
        state.setRawRotorPosition(turretRot * ShooterConstants.kTurretRatio);
        state.setRotorVelocity(m_sim.getAngularVelocity() / (2 * Math.PI) * ShooterConstants.kTurretRatio);
    }
}
