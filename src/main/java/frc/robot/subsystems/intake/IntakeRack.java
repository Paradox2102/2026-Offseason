package frc.robot.subsystems.intake;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.CanIds;
import frc.robot.health.HealthMonitor;
import frc.robot.lib.phoenix.PhoenixUtil;
import frc.robot.lib.util.AlertPublisher;
import frc.robot.lib.util.StallDetector;

/**
 * The intake rack: two motors, one per side, with no shaft between them.
 *
 * <p>Each side works on its own, because a hit can make one side skip teeth and not the other:
 * <ol>
 *   <li>MOTION: Motion Magic to 3.5 cm before the stop. (The old code made Talon 22 a Follower,
 *       which copies the leader's output, not its position, so the sides drifted.)
 *   <li>PUSH: a gentle push (6 A) into the stop until the side stops moving. Then its position is
 *       set to the stop ({@link RackSide}): this corrects skipped teeth.
 *   <li>REST: a light push (2 A) holds the side against the stop. A hit can push the intake in
 *       without the motors fighting it.
 *   <li>Pushed more than 2 cm away and still again: back to MOTION/PUSH.
 * </ol>
 * Positions are meters of rack travel, 0 = in.
 */
public class IntakeRack extends SubsystemBase {
    private enum Phase { MOTION, PUSH, REST }

    private final MotionMagicVoltage m_motionRequest = new MotionMagicVoltage(0);
    private final TorqueCurrentFOC m_currentRequest = new TorqueCurrentFOC(0);

    /** One side: its Talon, what it knows ({@link RackSide}) and what it is doing now. */
    private final class Side {
        final String m_label;
        final TalonFX m_talon;
        final RackSide m_state;
        final StallDetector m_stall = new StallDetector(
            IntakeConstants.kStallSpeed, IntakeConstants.kStallTime, IntakeConstants.kStallIgnore);
        final StallDetector m_settled = new StallDetector(IntakeConstants.kStallSpeed, IntakeConstants.kSettleTime, 0);
        final Alert m_rebootAlert;
        final Alert m_problemAlert;
        final Alert m_skipAlert;
        Phase m_phase = Phase.REST;
        boolean m_out = false;
        double m_pushStart = 0;
        int m_resets = 0;

        Side(String label, int canId, InvertedValue inverted) {
            m_label = label;
            m_talon = new TalonFX(canId, CanIds.k_canBus4);
            m_state = new RackSide(label, IntakeConstants.kRackStroke);
            m_rebootAlert = AlertPublisher.create("Health", "intake/" + canId + "/reboot",
                "Intake rack " + label + " rebooted: position copied from the other side until its next stop", Level.MEDIUM);
            m_problemAlert = AlertPublisher.create("Health", "intake/" + canId + "/stop", "", Level.MEDIUM);
            m_skipAlert = AlertPublisher.create("Health", "intake/" + canId + "/skip", "", Level.LOW);

            var config = new TalonFXConfiguration();
            config.MotorOutput.Inverted = inverted;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.Feedback.SensorToMechanismRatio = IntakeConstants.kRotationsPerMeter;
            config.Slot0.kS = IntakeConstants.kRackS;
            config.Slot0.kV = IntakeConstants.kRackV;
            config.Slot0.kP = IntakeConstants.kRackP;
            config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.kRackCruise;
            config.MotionMagic.MotionMagicAcceleration = IntakeConstants.kRackAcceleration;
            config.CurrentLimits.StatorCurrentLimit = IntakeConstants.kRackStatorLimit;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kRackSupplyLimit;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            PhoenixUtil.applyConfig("Intake rack " + label, () -> m_talon.getConfigurator().apply(config));
            m_talon.setPosition(0); // the robot boots with the intake in
            m_talon.hasResetOccurred(); // clear the "reset" flag of this boot
            HealthMonitor.register(m_talon);
        }

        double position() {
            return m_talon.getPosition().getValueAsDouble();
        }

        double velocity() {
            return m_talon.getVelocity().getValueAsDouble();
        }

        /** Go to a stop: Motion Magic first if far from it, then the push. */
        void start(boolean out, double now) {
            m_out = out;
            m_problemAlert.set(false);
            double distance = Math.abs(position() - m_state.stopPosition(out));
            if (distance > RackSide.kApproachDistance + IntakeConstants.kAtGoalTolerance) {
                m_phase = Phase.MOTION;
            } else {
                startPush(now);
            }
        }

        private void startPush(double now) {
            m_phase = Phase.PUSH;
            m_pushStart = now;
            m_stall.reset(now);
        }

        void update(double now) {
            double sign = m_out ? 1 : -1;
            switch (m_phase) {
                case MOTION -> {
                    double target = m_state.approachGoal(m_out);
                    m_talon.setControl(m_motionRequest.withPosition(target));
                    if (Math.abs(position() - target) < IntakeConstants.kAtGoalTolerance) {
                        startPush(now);
                    }
                }
                case PUSH -> {
                    m_talon.setControl(m_currentRequest.withOutput(sign * IntakeConstants.kPushCurrent));
                    if (m_stall.update(now, velocity())) {
                        // At the stop: now the position can be trusted again.
                        var result = m_state.stalledAt(m_out, position());
                        if (!Double.isNaN(result.setPosition())) {
                            m_talon.setPosition(result.setPosition());
                        }
                        if (result.alert() != null) {
                            m_skipAlert.setText(result.alert());
                            m_skipAlert.set(true);
                        }
                        m_rebootAlert.set(false);
                        rest(now);
                    } else if (now - m_pushStart > IntakeConstants.kPushTimeout) {
                        m_problemAlert.setText(m_state.pushTimedOut(m_out));
                        m_problemAlert.set(true);
                        rest(now);
                    }
                }
                case REST -> {
                    m_talon.setControl(m_currentRequest.withOutput(sign * IntakeConstants.kRestCurrent));
                    // Pushed away by a hit: do not fight it while it moves; once it is still, go back.
                    if (m_state.awayFromStop(m_out, position()) && m_settled.update(now, velocity())) {
                        start(m_out, now);
                    }
                }
            }
        }

        private void rest(double now) {
            m_phase = Phase.REST;
            m_settled.reset(now);
        }

        boolean atStop() {
            return m_phase == Phase.REST;
        }
    }

    // Talon 21 is on the turret (left) side; Talon 22 turns the other way on the right side.
    private final Side m_left = new Side("Talon 21 (left)", CanIds.intakeExtensionLeader, InvertedValue.CounterClockwise_Positive);
    private final Side m_right = new Side("Talon 22 (right)", CanIds.intakeExtensionFollower, InvertedValue.Clockwise_Positive);

    private final SkewGuard m_skewGuard = new SkewGuard(IntakeConstants.kSkewLimit, IntakeConstants.kSkewTime);
    private final Alert m_skewAlert = AlertPublisher.create("Health", "intake/skew",
        "Intake rack sides disagree by more than 2.5 cm (skipped teeth or binding): each side fixes itself at its stop", Level.LOW);
    private boolean m_goalOut = false;
    private boolean m_wasEnabled = false;
    private IntakeSim m_sim = null;

    public IntakeRack() {
        if (RobotBase.isSimulation()) {
            m_sim = new IntakeSim(m_left.m_talon, m_right.m_talon);
        }
    }

    @Override
    public void periodic() {
        double now = Timer.getTimestamp();
        checkReboot(m_left, m_right, now);
        checkReboot(m_right, m_left, now);

        // On enable, each side goes to the stop of the present goal (it may have been moved by
        // hand while disabled), and is re-trued there.
        boolean enabled = RobotState.isEnabled();
        if (enabled && !m_wasEnabled) {
            start(m_goalOut);
        }
        m_wasEnabled = enabled;

        // Only an alert: skipped teeth make the readings differ while the rack itself is straight.
        boolean moving = m_left.m_phase == Phase.MOTION && m_right.m_phase == Phase.MOTION;
        if (moving && m_skewGuard.update(now, m_left.position(), m_right.position())) {
            m_skewAlert.set(true);
        }

        // While disabled the Talons are neutral anyway: send nothing (at code start the devices may
        // not be up yet).
        if (enabled) {
            m_left.update(now);
            m_right.update(now);
        }
        log();
    }

    /** A Talon that rebooted reads 0 wherever it is: copy the other side's position, then re-true at a stop. */
    private void checkReboot(Side side, Side other, double now) {
        if (side.m_talon.hasResetOccurred()) {
            side.m_resets++;
            side.m_state.onReset();
            if (!other.m_state.isLost()) {
                side.m_talon.setPosition(other.position());
            }
            side.m_rebootAlert.set(true);
            side.start(m_goalOut, now);
        }
    }

    private void start(boolean out) {
        double now = Timer.getTimestamp();
        m_goalOut = out;
        m_skewGuard.reset();
        m_skewAlert.set(false);
        m_left.start(out, now);
        m_right.start(out, now);
    }

    /** Extend; ends when both sides rest at the out stop. The rack stays out after the command ends. */
    public Command extend() {
        return runOnce(() -> start(true)).andThen(Commands.waitUntil(this::atStop)).withName("Intake Extend");
    }

    /** Retract; ends when both sides rest at the in stop. */
    public Command retract() {
        return runOnce(() -> start(false)).andThen(Commands.waitUntil(this::atStop)).withName("Intake Retract");
    }

    private boolean atStop() {
        return m_left.atStop() && m_right.atStop();
    }

    /** Mean of the two sides, meters (0 = in). */
    public double getPosition() {
        return (m_left.position() + m_right.position()) / 2;
    }

    /** 0 = in, 1 = out. */
    public double getExtensionFraction() {
        double stroke = (m_left.m_state.outStop() + m_right.m_state.outStop()) / 2;
        return Math.clamp(getPosition() / stroke, 0.0, 1.0);
    }

    /** Simulator only: a hit pushes the left side in, and the right side skips teeth. */
    public void simHitAndSkip(double hitMeters, double skipMeters) {
        if (m_sim != null) {
            m_sim.hit(true, hitMeters);
            m_sim.skipTeeth(false, skipMeters);
        }
    }

    /** One line for test output: per side, position / out stop / phase / reboots. */
    public String status() {
        return describe(m_left) + " | " + describe(m_right);
    }

    private static String describe(Side side) {
        return String.format("%s %.3f m, out stop %.3f%s, %s, %d reboots", side.m_label, side.position(),
            side.m_state.outStop(), side.m_state.hasOutStop() ? " measured" : " (CAD)", side.m_phase, side.m_resets);
    }

    private void log() {
        Telemetry.log("Intake/Rack/Goal Out", m_goalOut);
        Telemetry.log("Intake/Rack/Position m", getPosition());
        Telemetry.log("Intake/Rack/Skew m", m_left.position() - m_right.position());
        Telemetry.log("Intake/Rack/At Stop", atStop());
        for (Side side : new Side[] {m_left, m_right}) {
            String prefix = "Intake/Rack/" + (side == m_left ? "Left" : "Right") + "/";
            Telemetry.log(prefix + "Position m", side.position());
            Telemetry.log(prefix + "Phase", side.m_phase.name());
            Telemetry.log(prefix + "Out Stop m", side.m_state.outStop());
            Telemetry.log(prefix + "Out Stop Measured", side.m_state.hasOutStop());
            Telemetry.log(prefix + "Lost", side.m_state.isLost());
            Telemetry.log(prefix + "Current A", side.m_talon.getStatorCurrent().getValueAsDouble());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (m_sim != null) {
            m_sim.update();
        }
    }
}
