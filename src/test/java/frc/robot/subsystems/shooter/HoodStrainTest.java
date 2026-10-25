package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class HoodStrainTest {
    private final HoodStrain m_strain = new HoodStrain(20, 2, 5, 0.5);

    /** Loops of 20 ms from {@code from} for {@code seconds}; true if it fired in any of them. */
    private boolean run(double from, double seconds, double amps, double errorDeg, double degPerSec) {
        boolean fired = false;
        for (double t = from; t < from + seconds; t += 0.02) {
            fired |= m_strain.update(t, amps, errorDeg, degPerSec);
        }
        return fired;
    }

    @Test
    void theFirstLoopAfterALongWaitDoesNotFire() {
        // Robot 2026-10-25: the hood homed 78 s after code start, still at ~20 A from the homing
        // push, with a 35 deg goal: the old debouncer fired at once.
        m_strain.reset();
        assertFalse(m_strain.update(78.02, 25, 35, 0));
    }

    @Test
    void pushingHardOnAStopForHalfASecondFires() {
        m_strain.reset();
        assertFalse(run(0.02, 0.4, 25, 35, 0));
        assertTrue(run(0.42, 0.2, 25, 35, 0));
    }

    @Test
    void pushingHardWhileMovingIsNormal() {
        // Speeding up toward a far goal takes current: not a stop.
        m_strain.reset();
        assertFalse(run(0.02, 2.0, 25, 20, 90));
    }

    @Test
    void anyBreakStartsTheTimeAgain() {
        m_strain.reset();
        run(0.02, 0.4, 25, 35, 0);
        m_strain.update(0.42, 5, 35, 0); // the current dropped once
        assertFalse(run(0.44, 0.4, 25, 35, 0));
    }

    @Test
    void atTheGoalItNeverFires() {
        m_strain.reset();
        assertFalse(run(0.02, 2.0, 25, 1, 0));
    }
}
