package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SkewGuardTest {
    private final SkewGuard m_guard = new SkewGuard(0.025, 0.1);

    @Test
    void smallSkewIsFine() {
        assertFalse(m_guard.update(0, 0.10, 0.12));
        assertFalse(m_guard.update(1, 0.10, 0.12));
    }

    @Test
    void tripsAfterTime() {
        assertFalse(m_guard.update(0, 0.10, 0.14));
        assertFalse(m_guard.update(0.05, 0.10, 0.14));
        assertTrue(m_guard.update(0.11, 0.10, 0.14));
        assertTrue(m_guard.update(0.12, 0.10, 0.10), "stays tripped until reset");
    }

    @Test
    void resetClears() {
        m_guard.update(0, 0.0, 0.1);
        m_guard.update(0.2, 0.0, 0.1);
        m_guard.reset();
        assertFalse(m_guard.update(0.3, 0.0, 0.0));
    }
}
