package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class DipCounterTest {
    private final DipCounter m_counter = new DipCounter(0.95, 0.98);

    @Test
    void aSpinUpFromRestIsNotADip() {
        // Below 95 % while it spins up: the wheel was not at speed yet.
        m_counter.update(0.0, 0, 100);
        m_counter.update(0.1, 40, 100);
        m_counter.update(0.2, 99, 100);
        assertEquals(0, m_counter.count());
        m_counter.update(0.3, 90, 100);
        assertEquals(1, m_counter.count(), "then a ball");
    }

    @Test
    void eachDropAndRecoveryIsOneBall() {
        double t = 0;
        for (int ball = 0; ball < 4; ball++) {
            m_counter.update(t, 100, 100);
            m_counter.update(t + 0.02, 93, 100);
            m_counter.update(t + 0.04, 90, 100); // still the same dip
            m_counter.update(t + 0.06, 96, 100); // not back yet
            m_counter.update(t + 0.08, 99, 100);
            t += 0.125;
        }
        assertEquals(4, m_counter.count());
        assertEquals(3 / 0.375, m_counter.rate(), 1e-9);
    }

    @Test
    void oneDipHasNoRate() {
        m_counter.update(0, 100, 100);
        m_counter.update(0.02, 90, 100);
        assertEquals(1, m_counter.count());
        assertTrue(Double.isNaN(m_counter.rate()));
    }

    @Test
    void resetStartsAgain() {
        m_counter.update(0, 100, 100);
        m_counter.update(0.02, 90, 100);
        m_counter.reset();
        assertEquals(0, m_counter.count());
    }
}
