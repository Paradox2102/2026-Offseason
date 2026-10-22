package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class RackSideTest {
    private static final double kStroke = 0.324;
    private final RackSide m_side = new RackSide("Test", kStroke);

    @Test
    void startsZeroedWithNoOutStop() {
        assertFalse(m_side.isLost());
        assertFalse(m_side.hasOutStop());
        assertEquals(kStroke, m_side.outStop(), 1e-9);
        assertEquals(kStroke - RackSide.kApproachDistance, m_side.approachGoal(true), 1e-9);
        assertEquals(RackSide.kApproachDistance, m_side.approachGoal(false), 1e-9);
    }

    @Test
    void firstOutStallMeasures() {
        var result = m_side.stalledAt(true, 0.330);
        assertTrue(result.atStop());
        assertTrue(Double.isNaN(result.setPosition()), "the first out stop is measured, not set");
        assertNull(result.alert());
        assertTrue(m_side.hasOutStop());
        assertEquals(0.330, m_side.outStop(), 1e-9);
    }

    @Test
    void laterOutStallRetrues() {
        m_side.stalledAt(true, 0.322);
        var small = m_side.stalledAt(true, 0.318); // 4 mm: normal
        assertEquals(0.322, small.setPosition(), 1e-9);
        assertNull(small.alert());
        var skipped = m_side.stalledAt(true, 0.305); // 1.7 cm: skipped teeth
        assertEquals(0.322, skipped.setPosition(), 1e-9);
        assertNotNull(skipped.alert());
    }

    @Test
    void inStallAlwaysZeroes() {
        var normal = m_side.stalledAt(false, 0.004);
        assertEquals(0.0, normal.setPosition(), 1e-9);
        assertNull(normal.alert());
        var skipped = m_side.stalledAt(false, -0.03);
        assertEquals(0.0, skipped.setPosition(), 1e-9);
        assertNotNull(skipped.alert());
    }

    @Test
    void lostSideHasNoSkipAlert() {
        m_side.onReset();
        assertTrue(m_side.isLost());
        var result = m_side.stalledAt(false, 0.05); // position copied from the other side
        assertEquals(0.0, result.setPosition(), 1e-9);
        assertNull(result.alert(), "a rebooted side is expected to be off");
        assertFalse(m_side.isLost());
    }

    @Test
    void pushedAwayFromTheStop() {
        m_side.stalledAt(true, 0.324);
        assertTrue(m_side.awayFromStop(true, 0.30));
        assertFalse(m_side.awayFromStop(true, 0.31));
        assertTrue(m_side.awayFromStop(false, 0.025));
        assertFalse(m_side.awayFromStop(false, 0.01));
    }

    @Test
    void pushTimeoutGivesAnAlert() {
        assertNotNull(m_side.pushTimedOut(true));
        assertFalse(m_side.hasOutStop(), "nothing measured: the next extend tries again");
    }

    @Test
    void farStallIsBlockedNotRetrued() {
        // Stopped 17 cm short (a ball or a wall): re-truing would be wrong by 17 cm.
        var result = m_side.stalledAt(true, 0.15);
        assertTrue(Double.isNaN(result.setPosition()));
        assertNotNull(result.alert());
        assertFalse(result.atStop());
        assertFalse(m_side.hasOutStop());
        m_side.stalledAt(true, 0.322);
        var blockedIn = m_side.stalledAt(false, 0.10);
        assertTrue(Double.isNaN(blockedIn.setPosition()));
        assertFalse(blockedIn.atStop());
    }

    @Test
    void stopTooShortIsNotMeasured() {
        // 2.4 cm short (blocked, or a small boot offset): outside the window, not stored.
        var result = m_side.stalledAt(true, 0.300);
        assertEquals(kStroke, result.setPosition(), 1e-9);
        assertNotNull(result.alert());
        assertFalse(m_side.hasOutStop());
    }

    @Test
    void lostSideDoesNotMeasureTheStroke() {
        m_side.onReset();
        var result = m_side.stalledAt(true, 0.318); // copied from the other side
        assertEquals(kStroke, result.setPosition(), 1e-9);
        assertNull(result.alert());
        assertFalse(m_side.hasOutStop());
        assertFalse(m_side.isLost());
    }
}
