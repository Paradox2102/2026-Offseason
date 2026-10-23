package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class PackPulseTest {
    // Pack 1 s after the first feed, pulse 0.3 s later; 2 cm at 4 Hz.
    private final PackPulse m_pack = new PackPulse(1.0, 0.3);

    private double at(double t) {
        return m_pack.offset(t, 0.02, 4);
    }

    @Test
    void noPackBeforeTheFirstFeed() {
        assertTrue(Double.isNaN(at(Double.NaN)));
    }

    @Test
    void noPackBeforeTheDelay() {
        assertTrue(Double.isNaN(at(0)));
        assertTrue(Double.isNaN(at(0.99)));
    }

    @Test
    void packPositionWhileTheRackGetsThere() {
        assertEquals(0, at(1.0), 1e-9);
        assertEquals(0, at(1.29), 1e-9);
    }

    @Test
    void thenASineAroundThePackPosition() {
        // 4 Hz: a quarter period is 1/16 s.
        assertEquals(0, at(1.3), 1e-9);
        assertEquals(0.02, at(1.3 + 1.0 / 16), 1e-9);
        assertEquals(0, at(1.3 + 2.0 / 16), 1e-9);
        assertEquals(-0.02, at(1.3 + 3.0 / 16), 1e-9);
        assertEquals(0.02, at(1.3 + 1.0 + 1.0 / 16), 1e-9, "still going a second later");
    }
}
