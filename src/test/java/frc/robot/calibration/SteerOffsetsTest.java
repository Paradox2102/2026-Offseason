package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class SteerOffsetsTest {
    @Test
    void aWheelThatReadsZeroKeepsItsOffset() {
        assertEquals(-0.2021, SteerOffsets.newOffset(-0.2021, 0), 1e-9);
    }

    @Test
    void theReadingIsTakenOff() {
        // Reads 0.1 turn with the wheel straight: the offset must be 0.1 turn less.
        assertEquals(0.018, SteerOffsets.newOffset(0.118, 0.1), 1e-9);
        assertEquals(-0.474 + 0.05, SteerOffsets.newOffset(-0.474, -0.05), 1e-9);
    }

    @Test
    void theResultWrapsToHalfATurn() {
        // -0.474 - 0.1 = -0.574, the same as +0.426.
        assertEquals(0.426, SteerOffsets.newOffset(-0.474, 0.1), 1e-9);
        assertEquals(-0.5, SteerOffsets.wrap(0.5), 1e-9);
        assertEquals(0.25, SteerOffsets.wrap(-1.75), 1e-9);
    }

    @Test
    void readingWithTheNewOffsetGivesZero() {
        double raw = 0.3371; // what the magnet reads with no offset, wheel straight
        double oldOffset = 0.3723144;
        double absolute = SteerOffsets.wrap(raw + oldOffset);
        double offset = SteerOffsets.newOffset(oldOffset, absolute);
        assertEquals(0, SteerOffsets.wrap(raw + offset), 1e-9);
    }
}
