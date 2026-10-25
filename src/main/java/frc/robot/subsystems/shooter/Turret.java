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
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.calibration.Calibration;
import frc.robot.constants.CanIds;
import frc.robot.health.HealthMonitor;
import frc.robot.lib.phoenix.PhoenixUtil;
import frc.robot.lib.util.AlertPublisher;

/**
 * The turret (Talon 44). Angles are turret rotations; 0 = the jig position (shooter to the
 * robot's right). Positive = counterclockwise seen from above. The mechanical limits allow 270
 * degrees each way (+-270 degrees); the Talon's soft limits hold the turret inside that.
 *
 * <p>The turret has no absolute sensor and no limit switch: only the motor's own encoder, which
 * counts from where the motor was at power-on. So the turret must be in its jig when the robot is
 * turned on. If the Talon reboots in a match, it counts from wherever the turret is then, so
 * every angle after that would be wrong: the turret stops and an alert asks for the jig and the
 * "Zero turret (in jig)" button.
 *
 * <p>Trim (Calibration, "Turret zero"): if the jig holds the turret a little off its true jig angle,
 * the Talon's reading there is the trim. Angles in the code are true angles: reading - trim.
 */
public class Turret extends SubsystemBase {
    private final TalonFX m_talon = new TalonFX(CanIds.shooterAzimuth, CanIds.k_canBus3);
    private final MotionMagicExpoTorqueCurrentFOC m_request = new MotionMagicExpoTorqueCurrentFOC(0);
    /** Enabled with no trusted angle: brake (the config's neutral mode is coast). */
    private final StaticBrake m_brake = new StaticBrake();
    private final Alert m_lostZeroAlert = AlertPublisher.create("Health", "shooter/turret/zero",
        "Turret Talon 44 rebooted: its zero is lost. Turret stopped: put it in the jig and press Pit > Zero turret", Level.HIGH);
    private final Alert m_restoredAlert = AlertPublisher.create("Health", "shooter/turret/restored", "", Level.MEDIUM);
    /** The last reading we trust (connected, no jump), rotations; NaN before the first one. */
    private double m_lastGood = Double.NaN;
    /** A reboot: give the Talon this position (retried until it takes it); NaN = nothing to do. */
    private double m_restore = Double.NaN;
    /** After a restore or a jig zero, the reading takes a few updates to show the new value. */
    private double m_settleUntil = Double.NEGATIVE_INFINITY;
    /** Jig zero asked for and not yet taken by the Talon. */
    private boolean m_jigZeroPending = false;
    private final double m_startTime = Timer.getTimestamp();
    private boolean m_zeroed = true; // in the jig at power-on
    private double m_goal = 0;
    /** The Talon's reading at the true jig angle, rotations (0 = the jig is right). */
    private double m_trim = 0;
    private static final double kSimPeriod = 0.005;
    private DCMotorSim m_sim = null;
    private Notifier m_simNotifier = null;

    /** The Talon configuration (static: unit tested). */
    static TalonFXConfiguration config() {
        var config = new TalonFXConfiguration();
        // A positive command turns the turret counterclockwise (seen from above). Checked on the
        // robot 2026-10-23: with CounterClockwise_Positive the 90 deg clockwise check turn went
        // counterclockwise, so the motor is Clockwise_Positive. The sim orientation below matches.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // Coast when neutral (user, 2026-10-25): disabled, the turret turns easily by hand. The
        // Talon still counts a turn by hand while it has power, so the angle stays right. Enabled,
        // the code never leaves it neutral: it holds a goal, or brakes (m_brake).
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
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
        return config;
    }

    public Turret() {
        var config = config();
        PhoenixUtil.applyConfig("Turret (44)", () -> m_talon.getConfigurator().apply(config));
        // No setPosition(0) here. A Talon reads 0 when it powers on, and the turret is in its jig
        // at power-on, so 0 = jig. A robot code restart (a redeploy, a crash) does not restart
        // the Talon: it still knows the true angle, and zeroing here would make wherever the
        // turret points the new 0 (and move the travel limits with it).
        m_talon.hasResetOccurred(); // clear the flag of this boot
        HealthMonitor.register(m_talon);
        double trimDeg = Calibration.get().turretTrimDeg;
        m_trim = Double.isNaN(trimDeg) ? 0 : trimDeg / 360;

        if (RobotBase.isSimulation()) {
            var motor = DCMotor.getKrakenX60Foc(1);
            m_sim = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(motor, 0.0497, ShooterConstants.kTurretRatio), motor);
            var state = m_talon.getSimState();
            state.Orientation = ChassisReference.Clockwise_Positive; // matches MotorOutput.Inverted
            state.setMotorType(TalonFXSimState.MotorType.KrakenX60);
            m_simNotifier = new Notifier(this::simStep);
            m_simNotifier.startPeriodic(kSimPeriod);
        }
    }

    @Override
    public void periodic() {
        double now = Timer.getTimestamp();
        boolean reset = m_talon.hasResetOccurred();
        if (reset && now - m_startTime > 3.0) {
            if (m_zeroed && !Double.isNaN(m_lastGood)) {
                // The Talon reads 0 again. Through 67.86:1 the turret hardly moves while its Talon
                // reboots, so its last good reading is still about right: give it back.
                m_restore = m_lastGood;
                m_restoredAlert.setText(String.format("Turret Talon 44 rebooted: position restored to %.1f deg"
                    + " from before the reboot. Zero it in the jig after the match", m_restore * 360));
                m_restoredAlert.set(true);
            } else {
                m_zeroed = false;
                m_lostZeroAlert.set(true);
            }
        }
        if (!Double.isNaN(m_restore) && m_talon.setPosition(m_restore).isOK()) {
            m_restore = Double.NaN;
            m_settleUntil = now + 0.1;
        }
        if (m_jigZeroPending && m_talon.setPosition(0).isOK()) {
            m_jigZeroPending = false;
            m_lastGood = 0;
            m_goal = 0;
            m_zeroed = true;
            m_settleUntil = now + 0.1;
            m_lostZeroAlert.set(false);
            m_restoredAlert.set(false);
        }
        trackLastGood(now);
        boolean restoring = !Double.isNaN(m_restore) || now < m_settleUntil;
        if (!RobotState.isEnabled()) {
            // Disabled: the Talon is neutral anyway; send nothing (at code start it may not be up).
        } else if (m_zeroed && !restoring) {
            m_talon.setControl(m_request.withPosition(m_goal + m_trim));
        } else {
            m_talon.setControl(m_brake);
        }
        Telemetry.log("Shooter/Turret/Goal deg", m_goal * 360);
        Telemetry.log("Shooter/Turret/Position deg", getPosition() * 360);
        Telemetry.log("Shooter/Turret/Error deg", (m_goal - getPosition()) * 360);
        Telemetry.log("Shooter/Turret/Zeroed", m_zeroed);
        Telemetry.log("Shooter/Turret/Trim deg", m_trim * 360);
    }

    /** Keep the last reading that can be trusted: from a connected Talon, with no reboot jump. */
    private void trackLastGood(double now) {
        if (!m_talon.isConnected() || !Double.isNaN(m_restore) || now < m_settleUntil || m_jigZeroPending) {
            return;
        }
        double position = rawPosition();
        if (Double.isNaN(m_lastGood) || Math.abs(position - m_lastGood) < ShooterConstants.kTurretMaxStepRot) {
            m_lastGood = position;
        }
    }

    /** Turret angle, rotations from the jig (true: the trim taken off). */
    public double getPosition() {
        return rawPosition() - m_trim;
    }

    /** The Talon's reading, rotations. */
    private double rawPosition() {
        return m_talon.getPosition().getValueAsDouble();
    }

    /** The trim in use, rotations. */
    public double getTrim() {
        return m_trim;
    }

    /** A new trim (the reading at the true jig angle), rotations; used at once. */
    public void setTrim(double rotations) {
        m_trim = rotations;
    }

    public boolean isZeroed() {
        return m_zeroed;
    }

    /** A jig zero was asked for, and the Talon has not taken it yet. */
    public boolean isJigZeroPending() {
        return m_jigZeroPending;
    }

    /** Zeroed, not restoring after a reboot, and within the tolerance of {@code goal}. */
    public boolean isOnTarget(double goal) {
        return isOnTarget(goal, ShooterConstants.kTurretToleranceDeg);
    }

    /** Zeroed, on CAN (a lost Talon's last reading stays frozen), and within {@code toleranceDeg}. */
    public boolean isOnTarget(double goal, double toleranceDeg) {
        boolean restoring = !Double.isNaN(m_restore) || Timer.getTimestamp() < m_settleUntil;
        return m_zeroed && !restoring && m_talon.isConnected() && Math.abs(getPosition() - goal) * 360 < toleranceDeg;
    }

    /** Point the turret at the angle from the supplier, every loop, until interrupted. */
    public Command track(DoubleSupplier goal) {
        return run(() -> m_goal = goal.getAsDouble()).withName("Turret Track");
    }

    /**
     * The turret is in its jig now: this is 0 again. The Talon may not take the new position at
     * once (it may still be booting), so periodic() tries until it does.
     */
    public Command zeroInJig() {
        return Commands.runOnce(() -> m_jigZeroPending = true).ignoringDisable(true).withName("Zero turret (in jig)");
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
