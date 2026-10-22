package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class JoystickShapingTest {
    @Test
    void insideDeadbandIsZero() {
        assertEquals(0, JoystickShaping.shape(0.05, 0.1, 2), 1e-9);
        assertEquals(0, JoystickShaping.shape(-0.09, 0.1, 2), 1e-9);
    }

    @Test
    void fullStickIsFullOutput() {
        assertEquals(1, JoystickShaping.shape(1, 0.1, 2), 1e-9);
        assertEquals(-1, JoystickShaping.shape(-1, 0.1, 3), 1e-9);
    }

    @Test
    void outputStartsAtZeroJustOutsideDeadband() {
        // The deadband is removed first, so there is no jump at its edge.
        assertEquals(0, JoystickShaping.shape(0.1000001, 0.1, 2), 1e-6);
    }

    @Test
    void curveKeepsSign() {
        // Halfway between the deadband edge and full stick: 0.5 squared = 0.25.
        assertEquals(0.25, JoystickShaping.shape(0.55, 0.1, 2), 1e-9);
        assertEquals(-0.25, JoystickShaping.shape(-0.55, 0.1, 2), 1e-9);
    }

    @Test
    void stickPastOneIsClamped() {
        assertEquals(1, JoystickShaping.shape(1.2, 0.1, 2), 1e-9);
    }
}
