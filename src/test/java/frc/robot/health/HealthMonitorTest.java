package frc.robot.health;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.util.HalHandleException;

class HealthMonitorTest {
    @Test
    void aValueTheRobotDoesNotHaveTurnsTheCheckOff() {
        // This SystemCore image throws HAL -1098 for getCommsDisableCount and others.
        int[] runs = {0};
        Runnable check = () -> {
            runs[0]++;
            throw new HalHandleException(" Code: -1098. HAL: A handle parameter was passed incorrectly");
        };
        assertFalse(HealthMonitor.runCheck("test no value", check));
        assertFalse(HealthMonitor.runCheck("test no value", check));
        assertEquals(1, runs[0], "skipped after the first failure");
    }

    @Test
    void otherFailuresRunAgainNextTime() {
        int[] runs = {0};
        Runnable check = () -> {
            runs[0]++;
            throw new IllegalStateException("no data yet");
        };
        assertFalse(HealthMonitor.runCheck("test other failure", check));
        assertFalse(HealthMonitor.runCheck("test other failure", check));
        assertEquals(2, runs[0]);
    }

    @Test
    void oneFailedCheckDoesNotStopTheNext() {
        HealthMonitor.runCheck("test first", () -> {
            throw new HalHandleException("Code: -1098");
        });
        boolean[] ran = {false};
        assertTrue(HealthMonitor.runCheck("test second", () -> ran[0] = true));
        assertTrue(ran[0]);
    }
}
