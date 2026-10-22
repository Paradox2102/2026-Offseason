package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.constants.CanIds;
import frc.robot.health.HealthMonitor;
import frc.robot.lib.phoenix.PhoenixUtil;
import frc.robot.lib.util.AlertPublisher;
import frc.robot.lib.util.StallDetector;

/**
 * The hood (Talon 41). Angles are degrees above its lower stop; up is positive.
 *
 * <p>Zero: the hood finds its lower stop by itself. On the first enable, and after a Talon reboot,
 * it pushes down gently until it stops moving, and calls that 0. Until then it does not move to
 * any goal. Stowed (0) it fits under the trench.
 */
public class Hood extends SubsystemBase {
    private final TalonFX m_talon = new TalonFX(CanIds.hood, CanIds.k_canBus3);
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private final TorqueCurrentFOC m_homeRequest = new TorqueCurrentFOC(ShooterConstants.kHoodHomeCurrent);
    private final StallDetector m_stall = new StallDetector(0.002, 0.15, 0.2); // hood rps
    private final double m_startTime = Timer.getTimestamp();
    private final Debouncer m_strainDebouncer = new Debouncer(0.5, DebounceType.RISING);
    private final Alert m_ratioAlert = AlertPublisher.create("Health", "shooter/hood/strain",
        "Hood pushes hard short of its goal: it is on a stop. Check ShooterConstants.kHoodRatio", Level.MEDIUM);
    private boolean m_homed = false;
    private boolean m_homing = false;
    private double m_goalDeg = 0;
    private SingleJointedArmSim m_sim = null;

    public Hood() {
        var config = new TalonFXConfiguration();
        // Check on the robot: a positive command must raise the hood.
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.kHoodRatio;
        config.Slot0.kS = ShooterConstants.kHoodS;
        config.Slot0.kP = ShooterConstants.kHoodP;
        config.Slot0.kG = ShooterConstants.kHoodG;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kHoodCruise;
        config.MotionMagic.MotionMagicAcceleration = ShooterConstants.kHoodAcceleration;
        // Gravity: Arm_Cosine takes position 0 as level. Set the hood's angle above level at its
        // lower stop here (rotations) before giving kG a value.
        config.Slot0.GravityArmPositionOffset = ShooterConstants.kHoodStopAboveLevelRot;
        // Upper limit in the Talon too (it scales with kHoodRatio, so it is only as right as that).
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (ShooterConstants.kHoodMaxDeg + 1) / 360;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kHoodStatorLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        PhoenixUtil.applyConfig("Hood (41)", () -> m_talon.getConfigurator().apply(config));
        m_talon.hasResetOccurred();
        HealthMonitor.register(m_talon);

        if (RobotBase.isSimulation()) {
            // Starts 3 degrees up, so the first enable has to find the stop.
            // No gravity in the sim: the hood must stay up until the homing push finds the stop.
            m_sim = new SingleJointedArmSim(DCMotor.getKrakenX44Foc(1), ShooterConstants.kHoodRatio, 0.0264, 0.209,
                0, Math.toRadians(ShooterConstants.kHoodMaxDeg + 2), false, Math.toRadians(3));
            var state = m_talon.getSimState();
            state.Orientation = ChassisReference.CounterClockwise_Positive;
            state.setMotorType(TalonFXSimState.MotorType.KrakenX44);
        }
    }

    @Override
    public void periodic() {
        double now = Timer.getTimestamp();
        if (m_talon.hasResetOccurred() && now - m_startTime > 3.0) {
            m_homed = false; // the Talon counts from 0 again: find the stop again
        }
        if (RobotState.isEnabled()) {
            if (!m_homed) {
                if (!m_homing) {
                    m_homing = true;
                    m_stall.reset(now);
                }
                m_talon.setControl(m_homeRequest);
                if (m_stall.update(now, m_talon.getVelocity().getValueAsDouble())) {
                    m_talon.setPosition(0);
                    m_homed = true;
                    m_homing = false;
                }
            } else {
                m_talon.setControl(m_request.withPosition(m_goalDeg / 360));
                // Pushing hard but not at the goal: the hood is on a stop. Most likely kHoodRatio is
                // wrong (the angles are then wrong too).
                boolean straining = m_talon.getStatorCurrent().getValueAsDouble() > 20
                    && Math.abs(getAngleDeg() - m_goalDeg) > 2;
                m_ratioAlert.set(m_strainDebouncer.calculate(straining) || m_ratioAlert.get());
            }
        } else {
            m_homing = false; // a push cut short by a disable starts again
        }
        Telemetry.log("Shooter/Hood/Goal deg", m_goalDeg);
        Telemetry.log("Shooter/Hood/Position deg", getAngleDeg());
        Telemetry.log("Shooter/Hood/Homed", m_homed);
    }

    public double getAngleDeg() {
        return m_talon.getPosition().getValueAsDouble() * 360;
    }

    public boolean isOnTarget(double goalDeg) {
        return isOnTarget(goalDeg, ShooterConstants.kHoodToleranceDeg);
    }

    public boolean isOnTarget(double goalDeg, double toleranceDeg) {
        return m_homed && m_talon.isConnected() && Math.abs(getAngleDeg() - goalDeg) < toleranceDeg;
    }

    /** Follow the angle from the supplier (clamped to the hood's range) until interrupted. */
    public Command aim(DoubleSupplier goalDeg) {
        return run(() -> m_goalDeg = Math.clamp(goalDeg.getAsDouble(), 0.0, ShooterConstants.kHoodMaxDeg)).withName("Hood Aim");
    }

    /** Down to the stop (to fit under the trench). */
    public Command stow() {
        return run(() -> m_goalDeg = 0).withName("Hood Stow");
    }

    @Override
    public void simulationPeriodic() {
        var state = m_talon.getSimState();
        state.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_sim.setInputVoltage(state.getMotorVoltage());
        m_sim.update(0.020);
        double hoodRot = m_sim.getAngle() / (2 * Math.PI);
        state.setRawRotorPosition(hoodRot * ShooterConstants.kHoodRatio);
        state.setRotorVelocity(m_sim.getVelocity() / (2 * Math.PI) * ShooterConstants.kHoodRatio);
    }
}
