package frc.robot.subsystems.intake;

import java.util.ArrayDeque;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
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
 * The intake rollers (Talon 20). Speeds are roller rotations per second.
 *
 * <p>Jam recovery: when the rollers should spin but stall with a high current (a game piece stuck
 * across them), they run out for a moment and then in again. Three jams in a few seconds raise
 * an alert, because then something is wrong that a short reverse cannot fix.
 */
public class IntakeRoller extends SubsystemBase {
    private final TalonFX m_talon = new TalonFX(CanIds.intakeRoller, CanIds.k_canBus4);
    private final MotionMagicVelocityVoltage m_velocityRequest = new MotionMagicVelocityVoltage(0);
    private final StaticBrake m_brake = new StaticBrake();

    private final Debouncer m_jamDebouncer = new Debouncer(IntakeConstants.kJamTime, DebounceType.RISING);
    private final ArrayDeque<Double> m_jamTimes = new ArrayDeque<>();
    private final Alert m_jamAlert = AlertPublisher.create("Health", "intake/jams",
        "Intake rollers jam again and again: check for a stuck game piece or a broken belt", Level.MEDIUM);
    private double m_target = 0;
    private double m_unjamUntil = Double.NEGATIVE_INFINITY;

    private FlywheelSim m_sim = null;

    public IntakeRoller() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = IntakeConstants.kRollerRatio;
        config.Slot0.kS = IntakeConstants.kRollerS;
        config.Slot0.kV = IntakeConstants.kRollerV;
        config.Slot0.kP = IntakeConstants.kRollerP;
        config.MotionMagic.MotionMagicAcceleration = IntakeConstants.kRollerAcceleration;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kRollerSupplyLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        PhoenixUtil.applyConfig("Intake rollers (20)", () -> m_talon.getConfigurator().apply(config));
        HealthMonitor.register(m_talon);

        if (RobotBase.isSimulation()) {
            var simState = m_talon.getSimState();
            simState.Orientation = ChassisReference.Clockwise_Positive;
            simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
            m_sim = new FlywheelSim(Models.flywheelFromPhysicalConstants(
                DCMotor.getKrakenX60Foc(1), IntakeConstants.kRollerMoi, IntakeConstants.kRollerRatio), DCMotor.getKrakenX60Foc(1));
        }
    }

    /** Run the rollers in (with jam recovery) until the command ends; then brake. */
    public Command runIn() {
        return startRun(this::resetJamCheck, () -> spin(IntakeConstants.kRollerInSpeed, true))
            .finallyDo(this::stop).withName("Intake Rollers In");
    }

    /** A new press starts clean: no leftover unjam, and the jam check waits its full time. */
    private void resetJamCheck() {
        m_unjamUntil = Double.NEGATIVE_INFINITY;
        m_jamDebouncer.calculate(false);
    }

    /** Run the rollers out until the command ends; then brake. */
    public Command runOut() {
        return runEnd(() -> spin(IntakeConstants.kRollerOutSpeed, false), this::stop).withName("Intake Rollers Out");
    }

    private void spin(double speed, boolean unjam) {
        double now = Timer.getTimestamp();
        double velocity = m_talon.getVelocity().getValueAsDouble();
        double current = m_talon.getStatorCurrent().getValueAsDouble();
        boolean stalled = Math.abs(velocity) < IntakeConstants.kJamSpeedFraction * Math.abs(speed)
            && current > IntakeConstants.kJamCurrent;
        if (now < m_unjamUntil) {
            // Running out: keep the jam check at "not stalled", so after the unjam it waits its
            // full time again before it can call a new jam.
            m_jamDebouncer.calculate(false);
        } else if (unjam && m_jamDebouncer.calculate(stalled)) {
            m_unjamUntil = now + IntakeConstants.kUnjamTime;
            m_jamTimes.addLast(now);
        }

        m_target = now < m_unjamUntil ? IntakeConstants.kRollerOutSpeed : speed;
        m_talon.setControl(m_velocityRequest.withVelocity(m_target));
    }

    private void stop() {
        m_target = 0;
        m_talon.setControl(m_brake);
    }

    /** The speed the rollers are asked for, roller rps (positive = in). */
    public double getTargetSpeed() {
        return m_target;
    }

    @Override
    public void periodic() {
        // Old jams leave the window here, so the count and the alert also update after the
        // trigger is released.
        double now = Timer.getTimestamp();
        while (!m_jamTimes.isEmpty() && now - m_jamTimes.peekFirst() > IntakeConstants.kJamWindow) {
            m_jamTimes.removeFirst();
        }
        m_jamAlert.set(m_jamTimes.size() >= IntakeConstants.kJamAlertCount);
        Telemetry.log("Intake/Roller/Target rps", m_target);
        Telemetry.log("Intake/Roller/Speed rps", m_talon.getVelocity().getValueAsDouble());
        Telemetry.log("Intake/Roller/Current A", m_talon.getStatorCurrent().getValueAsDouble());
        Telemetry.log("Intake/Roller/Unjamming", Timer.getTimestamp() < m_unjamUntil);
        Telemetry.log("Intake/Roller/Recent Jams", m_jamTimes.size());
    }

    @Override
    public void simulationPeriodic() {
        var simState = m_talon.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_sim.setInputVoltage(simState.getMotorVoltage());
        m_sim.update(0.020);
        // FlywheelSim gives the roller (output) speed in rad/s; the Talon sees the motor side.
        double rollerRps = m_sim.getAngularVelocity() / (2 * Math.PI);
        simState.setRotorVelocity(rollerRps * IntakeConstants.kRollerRatio);
        simState.addRotorPosition(rollerRps * IntakeConstants.kRollerRatio * 0.020);
    }
}
