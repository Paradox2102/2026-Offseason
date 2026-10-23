package frc.robot.subsystems.intake;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
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

import frc.robot.calibration.Calibration;
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
 *   <li>PUSH: a push (12 A net) into the stop until the side stops moving. Then its position is
 *       set to the stop ({@link RackSide}): this corrects skipped teeth.
 *   <li>REST: a light push (2 A net) holds the side against the stop. A hit can push the intake in
 *       without the motors fighting it.
 *   <li>Pushed more than 2 cm away and still again: back to MOTION/PUSH.
 * </ol>
 * One exception, while shooting ({@link #packWhile}): POSITION holds each side at a Motion Magic
 * goal inside its stroke, to pack the hopper. It has no push and no stall check, never sets the
 * Talon's position and never counts the side as at its stop: only a move back to a stop does.
 * Positions are meters of rack travel along its 20 degree slope, 0 = in. Gravity pulls the rack
 * out; every request includes a term against it, so the currents named here are the net push.
 */
public class IntakeRack extends SubsystemBase {
    private enum Phase { MOTION, PUSH, REST, RESYNC, POSITION }

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
        /** MOTION that stops moving short of its target: blocked, or the stop is nearer than the reading says. */
        final StallDetector m_motionStall = new StallDetector(
            IntakeConstants.kStallSpeed, IntakeConstants.kStallTime, IntakeConstants.kStallIgnore);
        final Alert m_rebootAlert;
        final Alert m_problemAlert;
        final Alert m_skipAlert;
        Phase m_phase = Phase.REST;
        boolean m_out = false;
        double m_pushStart = 0;
        int m_resets = 0;
        /** True after a push has put the side at its stop; false while moving or after a problem. */
        boolean m_confirmed = false;
        /** No new try before this time (after a blocked or timed-out push). */
        double m_retryAfter = Double.NEGATIVE_INFINITY;
        /** Position to give a rebooted Talon (RESYNC tries until the Talon takes it). */
        double m_resyncValue = Double.NaN;
        /** POSITION: the Motion Magic goal, meters. */
        double m_holdGoal = 0;

        /** The reading when the last push stalled, before any fix (to measure the out stops). */
        double m_lastStall = Double.NaN;
        /** Simulator only: this Talon does not answer. */
        boolean m_simDead = false;
        final Debouncer m_deadDebounce = new Debouncer(IntakeConstants.kDeadTime, DebounceType.RISING);
        boolean m_dead = false;

        Side(String label, int canId, InvertedValue inverted, double outStop) {
            m_label = label;
            m_talon = new TalonFX(canId, CanIds.k_canBus4);
            m_state = new RackSide(label, outStop);
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
            config.Slot0.kA = IntakeConstants.kRackA;
            // Gravity pulls the rack out (down its slope) with the same force everywhere: a
            // constant kG pulls in against it (negative = toward in).
            config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            config.Slot0.kG = -IntakeConstants.kGravityVolts;
            config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.kRackCruise;
            config.MotionMagic.MotionMagicAcceleration = IntakeConstants.kRackAcceleration;
            config.CurrentLimits.StatorCurrentLimit = IntakeConstants.kRackStatorLimit;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kRackSupplyLimit;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            PhoenixUtil.applyConfig("Intake rack " + label, () -> m_talon.getConfigurator().apply(config));
            // No setPosition(0) here: a Talon reads 0 when it powers on, and the robot powers on
            // with the intake in. A robot code restart does not restart the Talon, which still
            // knows where the rack is; zeroing here would make the rack's present place "in".
            m_talon.hasResetOccurred(); // clear the "reset" flag of this boot
            HealthMonitor.register(m_talon);
        }

        double position() {
            return m_talon.getPosition().getValueAsDouble();
        }

        boolean connected() {
            return !m_simDead && m_talon.isConnected();
        }

        double velocity() {
            return m_talon.getVelocity().getValueAsDouble();
        }

        /** Go to a stop: Motion Magic first if far from it, then the push. */
        void start(boolean out, double now) {
            start(out, now, position());
        }

        /** @param position where the side is (given, when the Talon's reading is not up to date yet) */
        void start(boolean out, double now, double position) {
            if (m_phase == Phase.RESYNC) {
                m_out = out; // RESYNC starts the move when the Talon has its position
                return;
            }
            // Already resting at this stop: nothing to do (no delay for the rollers).
            if (m_confirmed && out == m_out && m_phase == Phase.REST && !m_state.awayFromStop(out, position)) {
                return;
            }
            m_out = out;
            m_confirmed = false;
            double distance = Math.abs(position - m_state.stopPosition(out));
            if (distance > RackSide.kApproachDistance + IntakeConstants.kAtGoalTolerance) {
                m_phase = Phase.MOTION;
                m_motionStall.reset(now);
            } else {
                startPush(now);
            }
        }

        /** The Talon rebooted: give it this position, then go to the stop of the goal to confirm it. */
        void resync(double value, boolean out) {
            m_resets++;
            m_state.onReset();
            m_rebootAlert.set(true);
            m_confirmed = false;
            m_out = out;
            m_resyncValue = value;
            m_phase = Phase.RESYNC;
        }

        /** Hold this position (Motion Magic, no push). Not at a stop: a later move to a stop confirms it again. */
        void hold(double goal) {
            if (m_phase == Phase.RESYNC) {
                return; // the Talon gets its position first
            }
            m_phase = Phase.POSITION;
            m_confirmed = false;
            m_holdGoal = goal;
        }

        private void startPush(double now) {
            m_phase = Phase.PUSH;
            m_pushStart = now;
            m_stall.reset(now);
        }

        void update(double now) {
            double sign = m_out ? 1 : -1;
            switch (m_phase) {
                case RESYNC -> {
                    // Send nothing until the Talon has its position (it may still be booting).
                    if (m_talon.setPosition(m_resyncValue).isOK()) {
                        double value = m_resyncValue;
                        m_resyncValue = Double.NaN;
                        m_phase = Phase.REST;
                        start(m_out, now, value); // the reading may not show the new value yet
                    }
                }
                case MOTION -> {
                    double target = m_state.approachGoal(m_out);
                    m_talon.setControl(m_motionRequest.withPosition(target));
                    if (Math.abs(position() - target) < IntakeConstants.kAtGoalTolerance) {
                        startPush(now);
                    } else if (m_motionStall.update(now, velocity())) {
                        // Not moving and not there: blocked, or the stop is nearer than the reading
                        // says. Do not hold 40 A against it: the gentle push takes over.
                        startPush(now);
                    }
                }
                case PUSH -> {
                    m_talon.setControl(m_currentRequest.withOutput(sign * IntakeConstants.kPushCurrent - IntakeConstants.kGravityAmps));
                    if (m_stall.update(now, velocity())) {
                        m_lastStall = position();
                        var result = m_state.stalledAt(m_out, m_lastStall);
                        if (!Double.isNaN(result.setPosition())) {
                            m_talon.setPosition(result.setPosition());
                        }
                        if (result.atStop()) {
                            // At the stop: the position can be trusted again.
                            m_confirmed = true;
                            m_problemAlert.set(false);
                            m_rebootAlert.set(false);
                            if (result.alert() != null) {
                                m_skipAlert.setText(result.alert());
                                m_skipAlert.set(true);
                            }
                        } else {
                            // Blocked: rest (do not fight it) and try again later.
                            m_problemAlert.setText(result.alert());
                            m_problemAlert.set(true);
                            m_retryAfter = now + IntakeConstants.kRetryDelay;
                        }
                        rest(now);
                    } else if (now - m_pushStart > IntakeConstants.kPushTimeout) {
                        m_problemAlert.setText(m_state.pushTimedOut(m_out));
                        m_problemAlert.set(true);
                        m_retryAfter = now + IntakeConstants.kRetryDelay;
                        rest(now);
                    }
                }
                case POSITION -> m_talon.setControl(m_motionRequest.withPosition(m_holdGoal));
                case REST -> {
                    m_talon.setControl(m_currentRequest.withOutput(sign * IntakeConstants.kRestCurrent - IntakeConstants.kGravityAmps));
                    // Pushed away by a hit: do not fight it while it moves; once it is still, go back.
                    // Not confirmed (a blocked or timed-out push): also try again, even if the side
                    // has drifted onto its stop by itself, so a push confirms it there.
                    boolean settled = m_settled.update(now, velocity());
                    boolean needsMove = m_state.awayFromStop(m_out, position()) || !m_confirmed;
                    if (needsMove && settled && now >= m_retryAfter) {
                        m_confirmed = false;
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

    // Talon 21 is on the turret (left) side; Talon 22 turns the other way on the right side. The
    // out stops measured on the robot (Calibration) are used over the constants.
    private final Side m_left = new Side("Talon 21 (left)", CanIds.intakeExtensionLeader, InvertedValue.CounterClockwise_Positive,
        measured(Calibration.get().intakeOutStopLeft, IntakeConstants.kOutStopLeft));
    private final Side m_right = new Side("Talon 22 (right)", CanIds.intakeExtensionFollower, InvertedValue.Clockwise_Positive,
        measured(Calibration.get().intakeOutStopRight, IntakeConstants.kOutStopRight));

    private final SkewGuard m_skewGuard = new SkewGuard(IntakeConstants.kSkewLimit, IntakeConstants.kSkewTime);
    private final Alert m_skewAlert = AlertPublisher.create("Health", "intake/skew",
        "Intake rack sides disagree by more than 2.5 cm (skipped teeth or binding): each side fixes itself at its stop", Level.LOW);
    private final Alert m_deadAlert = AlertPublisher.create("Health", "intake/dead", "", Level.HIGH);
    private boolean m_goalOut = false;
    private boolean m_wasEnabled = false;
    /** Reasons the rack must stay out (near a trench, shooting): while one is true it extends and ignores retract. */
    private record HoldOut(String reason, BooleanSupplier active) {}
    private final List<HoldOut> m_holdOuts = new ArrayList<>();
    /**
     * A pack: each side at {@code fraction} of its out stop, plus {@code offset} meters (the pulse).
     */
    public record Pack(double fraction, double offset) {}
    private Supplier<Pack> m_packSource = () -> null;
    private boolean m_packing = false;
    /** A pack goal stays this far inside each stop, meters: a position move never pushes a stop. */
    private static final double kPackMargin = 0.01;
    private final double m_startTime = Timer.getTimestamp();
    private IntakeSim m_sim = null;

    public IntakeRack() {
        if (RobotBase.isSimulation()) {
            m_sim = new IntakeSim(m_left.m_talon, m_right.m_talon);
        }
    }

    @Override
    public void periodic() {
        double now = Timer.getTimestamp();
        checkReboots(now);
        checkDead(now);

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

        // Something needs the intake out (a trench ahead, a shot): go out, whatever the last
        // button was.
        String heldBy = heldOutBy();
        if (enabled && heldBy != null && !m_goalOut) {
            start(true);
        }
        Telemetry.log("Intake/Rack/Held Out By", heldBy == null ? "" : heldBy);

        // While shooting: pack (and pulse). When it ends, each side goes back to its stop with the
        // normal move, which confirms it there.
        Pack pack = enabled && m_goalOut ? m_packSource.get() : null;
        if (pack != null) {
            m_left.hold(packGoal(m_left, pack));
            m_right.hold(packGoal(m_right, pack));
        } else if (m_packing) {
            start(m_goalOut);
        }
        m_packing = pack != null;

        // While disabled the Talons are neutral anyway: send nothing (at code start the devices may
        // not be up yet).
        if (enabled) {
            m_left.update(now);
            m_right.update(now);
        }
        log();
    }

    /**
     * A Talon that rebooted reads 0 wherever it is. One side: copy the other side's position
     * (the sides move together). Both sides (a brownout): assume each rested at its goal's stop.
     * Either way the side then goes to the stop of the goal and confirms its position there.
     */
    private void checkReboots(double now) {
        boolean left = m_left.m_talon.hasResetOccurred();
        boolean right = m_right.m_talon.hasResetOccurred();
        if (now - m_startTime < IntakeConstants.kStartupGrace) {
            return; // the devices' own boot after code start, not a reboot in a match
        }
        if (left && right) {
            m_left.resync(m_left.m_state.stopPosition(m_goalOut), m_goalOut);
            m_right.resync(m_right.m_state.stopPosition(m_goalOut), m_goalOut);
        } else if (left) {
            m_left.resync(m_right.m_state.isLost() ? m_left.m_state.stopPosition(m_goalOut) : m_right.position(), m_goalOut);
        } else if (right) {
            m_right.resync(m_left.m_state.isLost() ? m_right.m_state.stopPosition(m_goalOut) : m_left.position(), m_goalOut);
        }
        if (left || right) {
            m_skewGuard.reset();
        }
    }

    /** A Talon that does not answer for {@link IntakeConstants#kDeadTime}: its side's place is unknown. */
    private void checkDead(double now) {
        if (now - m_startTime < IntakeConstants.kStartupGrace) {
            return; // the devices are still booting
        }
        String dead = null;
        for (Side side : new Side[] {m_left, m_right}) {
            side.m_dead = side.m_deadDebounce.calculate(!side.connected());
            if (side.m_dead) {
                dead = dead == null ? side.m_label : dead + " and " + side.m_label;
            }
        }
        if (dead != null) {
            m_deadAlert.setText("Intake: " + dead + " not answering. Do not drive under a trench."
                + " Shots no longer wait for the intake");
        }
        m_deadAlert.set(dead != null);
    }

    /**
     * A rack Talon does not answer: nothing tells where its side is, so the intake can never
     * count as out. The shot then goes anyway (user, 2026-10-22: with a dead Talon we cannot
     * score from the field, so shoot the balls we have).
     */
    public boolean hasDeadTalon() {
        return m_left.m_dead || m_right.m_dead;
    }

    private void start(boolean out) {
        double now = Timer.getTimestamp();
        m_goalOut = out;
        m_skewGuard.reset();
        m_skewAlert.set(false);
        m_left.start(out, now);
        m_right.start(out, now);
    }

    /**
     * Extend; ends when both sides rest at the out stop (or the rack packs for a shot: it is as far
     * out as it may be then). The rack stays out after the command ends.
     */
    public Command extend() {
        return runOnce(() -> start(true)).andThen(Commands.waitUntil(() -> atStop() || m_packing)).withName("Intake Extend");
    }

    /**
     * Retract; ends when both sides rest at the in stop. Ignored while something holds the rack
     * out ({@link #holdOutWhile}): then it ends at once, with the rack still out.
     */
    public Command retract() {
        return runOnce(() -> {
            if (heldOutBy() == null) {
                start(false);
            }
        }).andThen(Commands.waitUntil(() -> atStop() || m_packing)).withName("Intake Retract");
    }

    /** Keep the rack out while {@code active} is true (extend if needed; retract is ignored). */
    public void holdOutWhile(String reason, BooleanSupplier active) {
        m_holdOuts.add(new HoldOut(reason, active));
    }

    /**
     * Pack while {@code source} gives a pack (null: no pack). Only while enabled with the goal out
     * (a shot holds the rack out).
     */
    public void packWhile(Supplier<Pack> source) {
        m_packSource = source;
    }

    private static double packGoal(Side side, Pack pack) {
        double stop = side.m_state.outStop();
        return Math.clamp(pack.fraction() * stop + pack.offset(), kPackMargin, stop - kPackMargin);
    }

    /** Packing for a shot: the sides hold a position inside the stroke. */
    public boolean isPacking() {
        return m_packing;
    }

    /** The goal is out and each side is at least at {@code fraction} of its out stop less {@code tolerance} meters. */
    public boolean atLeast(double fraction, double tolerance) {
        return m_goalOut
            && m_left.position() >= fraction * m_left.m_state.outStop() - tolerance
            && m_right.position() >= fraction * m_right.m_state.outStop() - tolerance;
    }

    /** The first reason that holds the rack out now, or null. */
    private String heldOutBy() {
        for (var h : m_holdOuts) {
            if (h.active().getAsBoolean()) {
                return h.reason();
            }
        }
        return null;
    }

    /**
     * Fully out: the goal is out, the rack does not pack, and each side is within
     * {@link IntakeConstants#kExtendedTolerance} of its out stop. Each side counts on its own: one
     * side pushed in by a hit is "not out".
     */
    public boolean isExtended() {
        return m_goalOut && !m_packing
            && m_left.position() >= m_left.m_state.outStop() - IntakeConstants.kExtendedTolerance
            && m_right.position() >= m_right.m_state.outStop() - IntakeConstants.kExtendedTolerance;
    }

    private boolean atStop() {
        return m_left.atStop() && m_right.atStop();
    }

    private static double measured(double value, double constant) {
        return Double.isNaN(value) ? constant : value;
    }

    /** New out stops (Calibration, "Intake zero"), meters; used at once. */
    public void setOutStops(double left, double right) {
        m_left.m_state.setOutStop(left);
        m_right.m_state.setOutStop(right);
    }

    /** A side's out stop in use, meters. */
    public double outStop(boolean left) {
        return (left ? m_left : m_right).m_state.outStop();
    }

    /** A side's reading at its last stall, before any fix, meters (NaN: none yet). */
    public double lastStall(boolean left) {
        return (left ? m_left : m_right).m_lastStall;
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

    /** Simulator only: Talon 22 (right) stops answering; its side gets no power and its reading freezes. */
    public void simKillRight() {
        if (m_sim != null) {
            m_right.m_simDead = true;
            m_sim.killRight();
        }
    }

    /** Simulator only: something blocks the right side at this position (NaN: nothing). */
    public void simBlockRight(double position) {
        if (m_sim != null) {
            m_sim.blockRight(position);
        }
    }

    /** Simulator only: something blocks the left side at this position (NaN: nothing). */
    public void simBlockLeft(double position) {
        if (m_sim != null) {
            m_sim.blockLeft(position);
        }
    }

    /** One line for test output: per side, position / out stop / phase / reboots. */
    public String status() {
        return describe(m_left) + " | " + describe(m_right);
    }

    private static String describe(Side side) {
        return String.format("%s %.3f m, out stop %.3f, %s%s, %d reboots%s", side.m_label, side.position(),
            side.m_state.outStop(), side.m_phase, side.m_confirmed ? " at stop" : "", side.m_resets,
            side.m_dead ? ", NOT ANSWERING" : "");
    }

    private void log() {
        Telemetry.log("Intake/Rack/Goal Out", m_goalOut);
        Telemetry.log("Intake/Rack/Position m", getPosition());
        Telemetry.log("Intake/Rack/Skew m", m_left.position() - m_right.position());
        Telemetry.log("Intake/Rack/At Stop", atStop());
        Telemetry.log("Intake/Rack/Extended", isExtended());
        Telemetry.log("Intake/Rack/Packing", m_packing);
        for (Side side : new Side[] {m_left, m_right}) {
            String prefix = "Intake/Rack/" + (side == m_left ? "Left" : "Right") + "/";
            Telemetry.log(prefix + "Position m", side.position());
            Telemetry.log(prefix + "Phase", side.m_phase.name());
            Telemetry.log(prefix + "Out Stop m", side.m_state.outStop());
            Telemetry.log(prefix + "Last Stall m", side.m_lastStall);
            Telemetry.log(prefix + "Connected", !side.m_dead);
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
