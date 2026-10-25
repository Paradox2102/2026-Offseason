package frc.robot.subsystems.intake;

import frc.robot.lib.util.StallDetector;

/**
 * What one side of the intake rack asks its motor for, each loop. No hardware: unit tested.
 *
 * <p>User, 2026-10-25: the rack must never stall its motors, and never hold itself in place. So
 * the motor only moves the rack, and lets go when the move is over:
 * <ul>
 *   <li><b>Home</b> (code start: the reading means nothing). Coast: gravity slides the rack out
 *       down its slope, with no motor current. Once it is still, it is on the out stop: that sets
 *       the reading ({@link RackSide}). Then to the goal.
 *   <li><b>Move</b>: Motion Magic to just short of the stop ({@link #kInStopMargin},
 *       {@link #kOutStopMargin}), never into it. There: let go. Not moving and not there (binding,
 *       something in the way): let go at once, with an alert, and no retry.
 *   <li><b>Let go</b>: out, coast (gravity lays the rack on its out stop, which also re-trues the
 *       reading there once); in, brake (it may creep out, and is left to: no hold).
 *   <li><b>Hold</b>: only to pack the hopper during a shot, at a goal inside the stroke.
 * </ul>
 */
public class RackMover {
    /** What the motor does. */
    public enum Kind { COAST, BRAKE, MOTION }

    /** @param target the Motion Magic goal, meters (NaN unless MOTION) */
    public record Out(Kind kind, double target) {}

    /** A move in ends this far out from the in stop, meters. */
    public static final double kInStopMargin = 0.015;
    /** A move out ends this far in from the out stop, meters (gravity does the rest). */
    public static final double kOutStopMargin = 0.005;

    private enum Mode { HOME, MOVE, IDLE, HOLD }

    private static final Out kCoast = new Out(Kind.COAST, Double.NaN);
    private static final Out kBrake = new Out(Kind.BRAKE, Double.NaN);

    private final RackSide m_side;
    private final StallDetector m_homeStill = new StallDetector(
        IntakeConstants.kStallSpeed, IntakeConstants.kSettleTime, IntakeConstants.kStallIgnore);
    private final StallDetector m_moveStall = new StallDetector(
        IntakeConstants.kStallSpeed, IntakeConstants.kStallTime, IntakeConstants.kStallIgnore);
    private final StallDetector m_restStill = new StallDetector(IntakeConstants.kStallSpeed, IntakeConstants.kSettleTime, 0);
    private Mode m_mode = Mode.HOME;
    private boolean m_homeStarted = false;
    private boolean m_goalOut = false;
    /** The goal the side was let go at (IDLE). */
    private boolean m_idleOut = false;
    private boolean m_reTrued = false;
    private double m_holdGoal = 0;
    private double m_setPosition = Double.NaN;
    private double m_lastRest = Double.NaN;
    private String m_alert = null;
    private String m_skipAlert = null;

    public RackMover(RackSide side) {
        m_side = side;
    }

    /** Go to a stop. Already let go there: nothing to do. Not homed yet: home first, then there. */
    public void start(boolean out, double now) {
        if (m_mode == Mode.IDLE && out == m_idleOut) {
            m_goalOut = out;
            return;
        }
        restart(out, now);
    }

    /** Go to a stop, even if already let go there (a rebooted Talon). */
    public void restart(boolean out, double now) {
        m_goalOut = out;
        m_alert = null;
        if (!m_side.isHomed()) {
            if (m_mode != Mode.HOME) {
                m_mode = Mode.HOME;
                m_homeStarted = false;
            }
            return;
        }
        m_mode = Mode.MOVE;
        m_moveStall.reset(now);
    }

    /** Pack: hold this position, inside the stroke. Not homed: nothing (it homes first). */
    public void hold(double goal) {
        if (!m_side.isHomed()) {
            return;
        }
        m_mode = Mode.HOLD;
        m_holdGoal = goal;
    }

    public Out update(double now, double position, double velocity) {
        switch (m_mode) {
            case HOME -> {
                if (!m_homeStarted) {
                    m_homeStarted = true;
                    m_homeStill.reset(now);
                }
                if (!m_homeStill.update(now, velocity)) {
                    return kCoast;
                }
                m_lastRest = position;
                m_setPosition = m_side.stalledAt(true, position).setPosition();
                m_mode = Mode.MOVE;
                m_moveStall.reset(now);
                return new Out(Kind.MOTION, target());
            }
            case MOVE -> {
                double target = target();
                if (Math.abs(position - target) < IntakeConstants.kAtGoalTolerance) {
                    return letGo(now);
                }
                if (m_moveStall.update(now, velocity)) {
                    m_alert = String.format("Intake %s blocked %.1f cm from its %s position: it let go (it does not push)."
                        + " Check it by hand for binding", m_side.name(), Math.abs(position - target) * 100,
                        m_goalOut ? "out" : "in");
                    return letGo(now);
                }
                return new Out(Kind.MOTION, target);
            }
            case HOLD -> {
                return new Out(Kind.MOTION, m_holdGoal);
            }
            case IDLE -> {
                if (m_idleOut && !m_reTrued && m_restStill.update(now, velocity)) {
                    // Resting on the out stop by gravity: as good as a push into it.
                    m_reTrued = true;
                    m_lastRest = position;
                    var result = m_side.stalledAt(true, position);
                    m_setPosition = result.setPosition();
                    if (result.alert() != null) {
                        if (result.atStop()) {
                            m_skipAlert = result.alert();
                        } else {
                            m_alert = result.alert();
                        }
                    }
                }
                return m_idleOut ? kCoast : kBrake;
            }
        }
        return kBrake;
    }

    private Out letGo(double now) {
        m_mode = Mode.IDLE;
        m_idleOut = m_goalOut;
        m_reTrued = false;
        m_restStill.reset(now);
        return m_idleOut ? kCoast : kBrake;
    }

    /** Where a move to the goal ends: just short of its stop. */
    private double target() {
        return m_goalOut ? m_side.outStop() - kOutStopMargin : kInStopMargin;
    }

    /** A new reading to give the Talon, once (NaN: none). */
    public double takeSetPosition() {
        double value = m_setPosition;
        m_setPosition = Double.NaN;
        return value;
    }

    public boolean isHomed() {
        return m_side.isHomed();
    }

    /** Let go after a move (done, or blocked). */
    public boolean isIdle() {
        return m_mode == Mode.IDLE;
    }

    /** A move that gave up, or a side resting short of its out stop (null: none). Cleared by the next move. */
    public String alert() {
        return m_alert;
    }

    /** Skipped teeth found and fixed at the out stop (null: none). */
    public String skipAlert() {
        return m_skipAlert;
    }

    /** The reading when the side last came to rest on the out stop, before any fix, meters (NaN: none). */
    public double lastRest() {
        return m_lastRest;
    }

    /** For telemetry. */
    public String mode() {
        return m_mode.name();
    }
}
