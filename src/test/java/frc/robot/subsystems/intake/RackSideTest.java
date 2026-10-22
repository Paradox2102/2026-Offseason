package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class RackSideTest {
    private static final double kOutStop = 0.324;
    private final RackSide m_side = new RackSide("Test", kOutStop);

    @Test
    void startsAtTheGivenOutStop() {
        assertFalse(m_side.isLost());
        assertEquals(kOutStop, m_side.outStop(), 1e-9);
        assertEquals(kOutStop - RackSide.kApproachDistance, m_side.approachGoal(true), 1e-9);
        assertEquals(RackSide.kApproachDistance, m_side.approachGoal(false), 1e-9);
    }

    @Test
    void outStallNearTheStopRetrues() {
        var short4mm = m_side.stalledAt(true, 0.320);
        assertEquals(kOutStop, short4mm.setPosition(), 1e-9);
        assertNull(short4mm.alert());
        assertTrue(short4mm.atStop());
        var past3mm = m_side.stalledAt(true, 0.327);
        assertEquals(kOutStop, past3mm.setPosition(), 1e-9);
        assertNull(past3mm.alert());
        assertTrue(past3mm.atStop());
    }

    @Test
    void pastTheOutStopIsSkippedTeeth() {
        // A hard stop cannot be passed: the reading is wrong (a hit pushed the side in and its gear skipped).
        var result = m_side.stalledAt(true, 0.345);
        assertEquals(kOutStop, result.setPosition(), 1e-9);
        assertNotNull(result.alert());
        assertTrue(result.atStop());
    }

    @Test
    void shortOfTheOutStopIsNotCorrected() {
        // 2.4 cm short: a ball may hold this side there (the frame flexes), so the reading may be right.
        var result = m_side.stalledAt(true, 0.300);
        assertTrue(Double.isNaN(result.setPosition()), "correcting it would make a short side look out");
        assertNotNull(result.alert());
        assertTrue(result.alert().contains("short"));
        assertFalse(result.atStop());
        var far = m_side.stalledAt(true, 0.15);
        assertTrue(Double.isNaN(far.setPosition()));
        assertFalse(far.atStop());
    }

    @Test
    void pastTheInStopIsSkippedTeeth() {
        var normal = m_side.stalledAt(false, 0.004);
        assertEquals(0.0, normal.setPosition(), 1e-9);
        assertNull(normal.alert());
        assertTrue(normal.atStop());
        var skipped = m_side.stalledAt(false, -0.03);
        assertEquals(0.0, skipped.setPosition(), 1e-9);
        assertNotNull(skipped.alert());
        assertTrue(skipped.atStop());
    }

    @Test
    void shortOfTheInStopIsNotCorrected() {
        var result = m_side.stalledAt(false, 0.03);
        assertTrue(Double.isNaN(result.setPosition()));
        assertNotNull(result.alert());
        assertFalse(result.atStop());
    }

    @Test
    void tooFarPastIsNotTrusted() {
        // 6.6 cm past the out stop: more than a skip; the reading or the stop is wrong.
        var result = m_side.stalledAt(true, 0.390);
        assertTrue(Double.isNaN(result.setPosition()));
        assertNotNull(result.alert());
        assertFalse(result.atStop());
    }

    @Test
    void rebootedSideFollowsTheSameRule() {
        m_side.onReset();
        assertTrue(m_side.isLost());
        // Copied from the other side: short of the stop is not trusted either.
        var shortStop = m_side.stalledAt(true, 0.300);
        assertTrue(Double.isNaN(shortStop.setPosition()));
        assertFalse(shortStop.atStop());
        assertTrue(m_side.isLost(), "still lost until it stops at a stop");
        // Past the stop: corrected, with no skip alert (a rebooted side is expected to be off).
        var past = m_side.stalledAt(true, 0.340);
        assertEquals(kOutStop, past.setPosition(), 1e-9);
        assertNull(past.alert());
        assertTrue(past.atStop());
        assertFalse(m_side.isLost());
    }

    @Test
    void pushedAwayFromTheStop() {
        assertTrue(m_side.awayFromStop(true, 0.30));
        assertFalse(m_side.awayFromStop(true, 0.31));
        assertTrue(m_side.awayFromStop(false, 0.025));
        assertFalse(m_side.awayFromStop(false, 0.01));
    }

    @Test
    void pushTimeoutGivesAnAlert() {
        assertNotNull(m_side.pushTimedOut(true));
    }
}
