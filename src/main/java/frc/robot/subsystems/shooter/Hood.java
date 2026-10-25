package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.calibration.Calibration;
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
 *
 * <p>The ratio and the angle at the lower stop measured on the robot (Calibration, "Hood zero") are
 * used over the constants.
 */
public class Hood extends SubsystemBase {
    private final TalonFX m_talon = new TalonFX(CanIds.hood, CanIds.k_canBus3);
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private final VoltageOut m_homeRequest = new VoltageOut(ShooterConstants.kHoodHomeVolts);
    /** Calibration: slowly up to the upper stop, past the soft limit (it is only as right as the ratio). */
    private final VoltageOut m_jogRequest = new VoltageOut(kJogVolts).withIgnoreSoftwareLimits(true);
    private static final double kJogVolts = 1.0;
    private final StallDetector m_stall = new StallDetector(0.002, 0.15, 0.2); // hood rps
    private final double m_startTime = Timer.getTimestamp();
    /** On a stop short of the goal: over 20 A, over 2 deg from the goal, under 5 deg/s, for 0.5 s. */
    private final HoodStrain m_strain = new HoodStrain(20, 2, 5, 0.5);
    private final Alert m_ratioAlert = AlertPublisher.create("Health", "shooter/hood/strain",
        "Hood pushes hard short of its goal and does not move: it is on a stop or blocked. Check ShooterConstants.kHoodRatio and the hood by hand", Level.MEDIUM);
    private boolean m_homed = false;
    private boolean m_homing = false;
    private double m_goalDeg = 0;
    private boolean m_jogging = false;
    private double m_ratio;
    private double m_stopAboveLevelRot;
    private SingleJointedArmSim m_sim = null;

    public Hood() {
        var calibration = Calibration.get();
        m_ratio = Double.isNaN(calibration.hoodRatio) ? ShooterConstants.kHoodRatio : calibration.hoodRatio;
        m_stopAboveLevelRot = Double.isNaN(calibration.hoodStopAboveLevelDeg)
            ? ShooterConstants.kHoodStopAboveLevelRot : calibration.hoodStopAboveLevelDeg / 360;
        applyConfig();
        m_talon.hasResetOccurred();
        HealthMonitor.register(m_talon);

        if (RobotBase.isSimulation()) {
            // Starts 3 degrees up, so the first enable has to find the stop.
            // No gravity in the sim: the hood must stay up until the homing push finds the stop.
            // The sim's hood has the CAD ratio (kHoodRatio), whatever the Talon is told.
            m_sim = new SingleJointedArmSim(DCMotor.getKrakenX44Foc(1), ShooterConstants.kHoodRatio, 0.0264, 0.209,
                0, Math.toRadians(ShooterConstants.kHoodMaxDeg + 2), false, Math.toRadians(3));
            var state = m_talon.getSimState();
            state.Orientation = ChassisReference.CounterClockwise_Positive;
            state.setMotorType(TalonFXSimState.MotorType.KrakenX44);
        }
    }

    private void applyConfig() {
        var config = new TalonFXConfiguration();
        // Check on the robot: a positive command must raise the hood.
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = m_ratio;
        config.Slot0.kS = ShooterConstants.kHoodS;
        config.Slot0.kP = ShooterConstants.kHoodP;
        config.Slot0.kG = ShooterConstants.kHoodG;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kHoodCruise;
        config.MotionMagic.MotionMagicAcceleration = ShooterConstants.kHoodAcceleration;
        // Gravity: Arm_Cosine takes position 0 as level. Set the hood's angle above level at its
        // lower stop here (rotations) before giving kG a value.
        config.Slot0.GravityArmPositionOffset = m_stopAboveLevelRot;
        // Upper limit in the Talon too (it scales with kHoodRatio, so it is only as right as that).
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (ShooterConstants.kHoodMaxDeg + 1) / 360;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kHoodStatorLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        PhoenixUtil.applyConfig("Hood (41)", () -> m_talon.getConfigurator().apply(config));
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
                // On the stop: zero there. The Talon may not take the new position at once; keep
                // pushing and try again next loop.
                if (m_stall.update(now, m_talon.getVelocity().getValueAsDouble()) && m_talon.setPosition(0).isOK()) {
                    m_homed = true;
                    m_homing = false;
                    m_strain.reset(); // the homing push's own current is not strain
                }
            } else if (m_jogging) {
                m_talon.setControl(m_jogRequest);
            } else {
                m_talon.setControl(m_request.withPosition(m_goalDeg / 360));
                // Pushing hard, not moving, not at the goal: the hood is on a stop. Most likely
                // kHoodRatio is wrong (the angles are then wrong too), or something blocks it.
                boolean straining = m_strain.update(now, m_talon.getStatorCurrent().getValueAsDouble(),
                    getAngleDeg() - m_goalDeg, m_talon.getVelocity().getValueAsDouble() * 360);
                m_ratioAlert.set(straining || m_ratioAlert.get());
            }
        } else {
            m_homing = false; // a push cut short by a disable starts again
        }
        Telemetry.log("Shooter/Hood/Goal deg", m_goalDeg);
        Telemetry.log("Shooter/Hood/Position deg", getAngleDeg());
        Telemetry.log("Shooter/Hood/Homed", m_homed);
        // To see what the hood does on the robot (2026-10-25 it stayed at 0 with a 35 deg goal).
        Telemetry.log("Shooter/Hood/Current A", m_talon.getStatorCurrent().getValueAsDouble());
        Telemetry.log("Shooter/Hood/Motor V", m_talon.getMotorVoltage().getValueAsDouble());
        Telemetry.log("Shooter/Hood/Speed deg per s", m_talon.getVelocity().getValueAsDouble() * 360);
    }

    public boolean isHomed() {
        return m_homed;
    }

    /** Find the lower stop again (Calibration). */
    public void rehome() {
        m_homed = false;
    }

    /** The ratio in use, motor turns per hood turn. */
    public double ratio() {
        return m_ratio;
    }

    /** Motor turns since the lower stop (whatever the ratio). */
    public double motorTurnsFromStop() {
        return m_talon.getPosition().getValueAsDouble() * m_ratio;
    }

    /**
     * A measured ratio and angle above level at the lower stop (Calibration, "Hood zero"). The
     * Talon config is applied again and the hood finds its lower stop again.
     */
    public void setCalibration(double ratio, double stopAboveLevelDeg) {
        m_ratio = ratio;
        m_stopAboveLevelRot = stopAboveLevelDeg / 360;
        applyConfig();
        m_homed = false;
    }

    /** Calibration: slowly up (1 V, past the soft limit) while held, to find the upper stop. */
    public Command jogUp() {
        return startEnd(() -> m_jogging = true, () -> m_jogging = false).withName("Hood Jog Up");
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
