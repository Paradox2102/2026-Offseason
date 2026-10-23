package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.system.Timer;

class VisionClockTest {
    @Test
    void convertsWithAPositiveOffset() {
        assertEquals(15.0, VisionClock.toDriveTime(10.0, 105.0, 100.0), 1e-9);
    }

    @Test
    void convertsWithANegativeOffset() {
        assertEquals(7.0, VisionClock.toDriveTime(10.0, 97.0, 100.0), 1e-9);
    }

    @Test
    void acceptsNormalLatency() {
        assertEquals("", VisionClock.ageRejectReason(0.035));
    }

    @Test
    void rejectsOldFrame() {
        assertEquals("old", VisionClock.ageRejectReason(0.6));
    }

    @Test
    void rejectsFutureFrame() {
        assertEquals("future", VisionClock.ageRejectReason(-0.02));
    }

    @Test
    void allowsSmallClockJitter() {
        assertEquals("", VisionClock.ageRejectReason(-0.005));
    }

    @Test
    void networkTablesClockIsInSeconds() {
        // Same epoch off the robot: a wrong unit is off by a factor of 1000 or more.
        assertEquals(Timer.getMonotonicTimestamp(), VisionClock.networkTablesNowSeconds(), 0.5);
    }
}
