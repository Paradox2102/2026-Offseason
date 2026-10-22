package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class StallDetectorTest {
    // Stalled = slower than 0.005 for 0.15 s, after the first 0.3 s.
    private final StallDetector m_detector = new StallDetector(0.005, 0.15, 0.3);

    @Test
    void ignoresTheFirstWindow() {
        m_detector.reset(10.0);
        // Not moving yet at the start of a push is normal: the motor has to speed up first.
        assertFalse(m_detector.update(10.0, 0));
        assertFalse(m_detector.update(10.2, 0));
        assertFalse(m_detector.update(10.29, 0));
    }

    @Test
    void stallsAfterHoldTime() {
        m_detector.reset(0);
        assertFalse(m_detector.update(0.3, 0.001));
        assertFalse(m_detector.update(0.4, 0.001));
        assertTrue(m_detector.update(0.46, 0.001));
    }

    @Test
    void movementRestartsTheClock() {
        m_detector.reset(0);
        m_detector.update(0.3, 0.001);
        m_detector.update(0.4, 0.5); // moving again
        assertFalse(m_detector.update(0.5, 0.001));
        assertTrue(m_detector.update(0.66, -0.002));
    }
}
