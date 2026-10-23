package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;

class DriveCommandsTest {
    private static double nearest(double deg) {
        return DriveCommands.nearestQuarterTurn(Rotation2d.fromDegrees(deg)).getDegrees();
    }

    @Test
    void snapsToTheNearestMultipleOf90() {
        assertEquals(0, nearest(10), 1e-9);
        assertEquals(90, nearest(80), 1e-9);
        assertEquals(90, nearest(130), 1e-9);
        assertEquals(180, Math.abs(nearest(170)), 1e-9);
        assertEquals(-90, nearest(-100), 1e-9);
        assertEquals(0, nearest(-20), 1e-9);
    }

    @Test
    void wrapsAroundPast180() {
        // 200 degrees is -160: nearest is 180.
        assertEquals(180, Math.abs(nearest(200)), 1e-9);
        assertEquals(-90, nearest(265), 1e-9);
        assertEquals(0, nearest(355), 1e-9);
    }
}
