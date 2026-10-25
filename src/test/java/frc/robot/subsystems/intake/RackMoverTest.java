package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.intake.RackMover.Kind;
import frc.robot.subsystems.intake.RackMover.Out;

class RackMoverTest {
    private static final double kOutStop = 0.324;
    private final RackMover m_mover = new RackMover(new RackSide("Test", kOutStop));
    private double m_now = 0;

    /** Run loops of 20 ms with the side still at {@code position}. */
    private Out still(double position, double seconds) {
        Out out = null;
        for (double t = 0; t < seconds; t += 0.02) {
            m_now += 0.02;
            out = m_mover.update(m_now, position, 0);
        }
        return out;
    }

    private Out moving(double position) {
        m_now += 0.02;
        return m_mover.update(m_now, position, 0.3);
    }

    /** Home at the out stop (goal out); after it the Talon reads the out stop, and the side is let go there. */
    private void homeAtOutStop() {
        m_mover.start(true, m_now);
        still(0.0, 0.6); // the Talon read 0 at power-on, at the out stop
        assertTrue(m_mover.isHomed());
        assertEquals(kOutStop, m_mover.takeSetPosition(), 1e-9);
        still(kOutStop, 0.1);
        assertTrue(m_mover.isIdle());
    }

    @Test
    void homingUsesNoMotorAtAllGravitySlidesTheRackOut() {
        m_mover.start(false, m_now);
        // Sliding out: coast, no current.
        assertEquals(Kind.COAST, moving(0.05).kind());
        assertEquals(Kind.COAST, moving(0.10).kind());
        // Still: that is the out stop.
        Out out = still(0.12, 0.6);
        assertTrue(m_mover.isHomed());
        assertEquals(kOutStop, m_mover.takeSetPosition(), 1e-9);
        // Then to the goal (in), by Motion Magic.
        assertEquals(Kind.MOTION, out.kind());
    }

    @Test
    void aMoveInStopsShortOfTheInStopAndThenLetsGo() {
        homeAtOutStop();
        m_mover.start(false, m_now);
        Out out = moving(0.2);
        assertEquals(Kind.MOTION, out.kind());
        assertEquals(RackMover.kInStopMargin, out.target(), 1e-9);
        assertTrue(out.target() > 0, "never into the hard stop");
        // There: brake, no hold current.
        out = still(RackMover.kInStopMargin, 0.1);
        assertEquals(Kind.BRAKE, out.kind());
        assertTrue(m_mover.isIdle());
    }

    @Test
    void aMoveOutStopsShortOfTheOutStopAndThenCoasts() {
        homeAtOutStop();
        m_mover.start(false, m_now);
        still(RackMover.kInStopMargin, 0.1);
        m_mover.start(true, m_now);
        Out out = moving(0.2);
        assertEquals(Kind.MOTION, out.kind());
        assertTrue(out.target() < kOutStop, "never into the hard stop");
        // There: coast, so gravity lays it on the out stop with no motor current.
        out = still(kOutStop - RackMover.kOutStopMargin, 0.1);
        assertEquals(Kind.COAST, out.kind());
    }

    @Test
    void aBlockedMoveGivesUpAtOnceAndDoesNotTryAgain() {
        homeAtOutStop();
        m_mover.start(false, m_now);
        // Stuck 25 cm out (binding): not moving and not there.
        Out out = still(0.25, 1.0);
        assertEquals(Kind.BRAKE, out.kind());
        assertNotNull(m_mover.alert());
        assertTrue(m_mover.alert().contains("blocked"), m_mover.alert());
        // No retry: it stays let go however long it waits.
        out = still(0.25, 5.0);
        assertEquals(Kind.BRAKE, out.kind());
    }

    @Test
    void itNeverAsksForACurrentPushOrAHoldAtAStop() {
        m_mover.start(true, m_now);
        double[] positions = {0.0, 0.0, 0.1, 0.2, 0.3, 0.3, 0.3};
        for (int cycle = 0; cycle < 4; cycle++) {
            m_mover.start(cycle % 2 == 0, m_now);
            for (double p : positions) {
                for (int i = 0; i < 30; i++) {
                    Out out = still(p, 0.02);
                    assertTrue(out.kind() != Kind.MOTION
                        || (out.target() >= RackMover.kInStopMargin - 1e-9 && out.target() <= kOutStop - RackMover.kOutStopMargin + 1e-9),
                        "Motion Magic target " + out.target() + " is at or past a stop");
                }
            }
        }
    }

    @Test
    void restingOnTheOutStopByGravityReTruesThePosition() {
        homeAtOutStop();
        // Skipped teeth: the reading is 2 cm past the out stop while it rests there.
        m_mover.start(true, m_now);
        still(kOutStop + 0.02, 1.0);
        assertEquals(kOutStop, m_mover.takeSetPosition(), 1e-9);
        assertTrue(Double.isNaN(m_mover.takeSetPosition()), "only once");
    }

    @Test
    void idleAtTheInPositionDoesNotChaseADrift() {
        homeAtOutStop();
        m_mover.start(false, m_now);
        still(RackMover.kInStopMargin, 0.1);
        // Gravity lets it creep out: it stays let go (user, 2026-10-25: no hold).
        Out out = still(0.08, 3.0);
        assertEquals(Kind.BRAKE, out.kind());
        assertNull(m_mover.alert());
        assertTrue(Double.isNaN(m_mover.takeSetPosition()), "no re-true away from the out stop");
    }

    @Test
    void aPackHoldIsAPositionInsideTheStroke() {
        homeAtOutStop();
        m_mover.hold(0.28);
        Out out = still(0.30, 0.04);
        assertEquals(Kind.MOTION, out.kind());
        assertEquals(0.28, out.target(), 1e-9);
    }
}
