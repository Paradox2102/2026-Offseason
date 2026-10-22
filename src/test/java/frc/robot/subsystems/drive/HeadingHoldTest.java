package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import frc.robot.commands.DriveCommands;
import frc.robot.constants.FieldConstants;

class HeadingHoldTest {
    private final HeadingHold m_hold = new HeadingHold(0.15);

    @Test
    void driverTurningGivesNoHold() {
        assertTrue(m_hold.update(0.5, 0.0, Rotation2d.fromDegrees(30), true).isEmpty());
    }

    @Test
    void holdCapturedAfterRateSettles() {
        // The driver lets go of the stick while the robot still turns: no hold yet, or the robot
        // would snap back to where it was when the stick was released.
        assertTrue(m_hold.update(0, 1.0, Rotation2d.fromDegrees(40), true).isEmpty());
        var held = m_hold.update(0, 0.05, Rotation2d.fromDegrees(45), true);
        assertTrue(held.isPresent());
        assertEquals(45, held.get().getDegrees(), 1e-9);
    }

    @Test
    void holdIsKeptWhileTranslating() {
        m_hold.update(0, 0.0, Rotation2d.fromDegrees(10), true);
        var held = m_hold.update(0, 0.3, Rotation2d.fromDegrees(14), true);
        assertEquals(10, held.get().getDegrees(), 1e-9);
    }

    @Test
    void noHoldWhenStoppedAndNotTranslating() {
        m_hold.update(0, 0.0, Rotation2d.fromDegrees(10), true);
        assertTrue(m_hold.update(0, 0.0, Rotation2d.fromDegrees(10), false).isEmpty());
    }

    @Test
    void resetClearsHold() {
        m_hold.update(0, 0.0, Rotation2d.fromDegrees(10), true);
        m_hold.reset();
        var held = m_hold.update(0, 0.0, Rotation2d.fromDegrees(20), true);
        assertEquals(20, held.get().getDegrees(), 1e-9);
    }

    @Test
    void hubAngleWithNoAlliance() {
        // With no alliance known the code aims at the blue hub. From a point straight
        // "below" the blue hub (smaller y), the hub is at +90 degrees.
        var hub = FieldConstants.hub(false);
        var pose = new Pose2d(hub.getX(), hub.getY() - 2, Rotation2d.ZERO);
        assertEquals(90, DriveCommands.angleToHub(pose, false).getDegrees(), 1e-9);
        assertFalse(Drive.isRedAlliance());
    }
}
