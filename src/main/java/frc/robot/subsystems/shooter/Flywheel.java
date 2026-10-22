package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.system.DCMotor;
import org.wpilib.telemetry.Telemetry;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.constants.CanIds;
import frc.robot.health.HealthMonitor;
import frc.robot.lib.phoenix.PhoenixUtil;
import frc.robot.lib.sim.RollerSim;

/**
 * The flywheel (Talons 42 and 43, one shaft). Speeds are wheel rps.
 *
 * <p>VelocityTorqueCurrentFOC (Pro) controls motor current, which is torque. When a ball takes
 * speed from the wheel, the error asks for more torque at once, the same at 12.5 V or 10 V, so
 * the wheel recovers faster than with a voltage loop. Talon 43 follows Talon 42, turning the other
 * way (the motors face each other).
 */
public class Flywheel extends SubsystemBase {
    private final TalonFX m_leader = new TalonFX(CanIds.shooterLeader, CanIds.k_canBus3);
    private final TalonFX m_follower = new TalonFX(CanIds.shooterFollower, CanIds.k_canBus3);
    private final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0);
    private final CoastOut m_coast = new CoastOut();
    private final Follower m_follow = new Follower(CanIds.shooterLeader, MotorAlignmentValue.Opposed);
    private final Debouncer m_ready = new Debouncer(ShooterConstants.kFlywheelReadyTime, DebounceType.RISING);
    private double m_target = 0;
    private boolean m_isReady = false;
    private RollerSim m_sim = null;

    public Flywheel() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.kFlywheelRatio;
        config.Slot0.kS = ShooterConstants.kFlywheelS;
        config.Slot0.kP = ShooterConstants.kFlywheelP;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kFlywheelStatorLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kFlywheelSupplyLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // Both Talons get the limits: a follower does not take the leader's config.
        PhoenixUtil.applyConfig("Flywheel leader (42)", () -> m_leader.getConfigurator().apply(config));
        PhoenixUtil.applyConfig("Flywheel follower (43)", () -> m_follower.getConfigurator().apply(config));
        HealthMonitor.register(m_leader);
        HealthMonitor.register(m_follower);

        if (RobotBase.isSimulation()) {
            m_sim = new RollerSim(m_leader, DCMotor.getKrakenX60Foc(2), ShooterConstants.kFlywheelRatio,
                ShooterConstants.kFlywheelMoi, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX60);
        }
    }

    @Override
    public void periodic() {
        if (RobotState.isEnabled()) {
            // Sent every loop: one request at start-up is lost if Talon 43 is not up yet.
            m_follower.setControl(m_follow);
        }
        double speed = getSpeed();
        m_isReady = m_ready.calculate(m_target > 0
            && Math.abs(speed - m_target) <= ShooterConstants.kFlywheelTolerance * m_target);
        Telemetry.log("Shooter/Flywheel/Target rps", m_target);
        Telemetry.log("Shooter/Flywheel/Speed rps", speed);
        Telemetry.log("Shooter/Flywheel/Ready", m_isReady);
    }

    /** Wheel speed, rps. */
    public double getSpeed() {
        return m_leader.getVelocity().getValueAsDouble();
    }

    /** At the target speed (within 3 %) for 0.1 s. */
    public boolean isReady() {
        return m_isReady;
    }

    /** Spin at the speed from the supplier until interrupted; then coast down. */
    public Command spin(DoubleSupplier rps) {
        return runEnd(() -> {
            m_target = rps.getAsDouble();
            m_leader.setControl(m_request.withVelocity(m_target));
        }, () -> {
            m_target = 0;
            m_leader.setControl(m_coast);
        }).withName("Flywheel Spin");
    }

    @Override
    public void simulationPeriodic() {
        m_sim.update();
    }
}
