package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import frc.robot.constants.FieldConstants;

class TrenchGuardTest {
    private static final ChassisVelocities kStill = new ChassisVelocities();

    private static boolean near(double x, double y, double vx, double vy) {
        return TrenchGuard.nearTrench(new Pose2d(x, y, Rotation2d.ZERO), new ChassisVelocities(vx, vy, 0));
    }

    @Test
    void underTheArmIsNear() {
        assertTrue(near(4.6, 0.7, 0, 0));
        assertTrue(near(4.6, 7.4, 0, 0), "the high-y trench too");
    }

    @Test
    void justOutsideTheReachIsNotNear() {
        assertTrue(near(4.435 - 0.85, 0.7, 0, 0));
        assertFalse(near(4.435 - 1.0, 0.7, 0, 0));
    }

    @Test
    void drivingTowardTheTrenchIsNearEarly() {
        // 1.5 m before the arm at 3 m/s: inside the reach within 0.6 s.
        assertFalse(near(2.9, 0.7, 0, 0));
        assertTrue(near(2.9, 0.7, 3.0, 0));
        assertFalse(near(2.9, 0.7, -3.0, 0), "driving away");
    }

    @Test
    void theBumpBesideTheTrenchIsNotNear() {
        // On the bump next to the trench wall: the wall separates it from the lane.
        assertFalse(near(4.6, 2.1, 0, 0));
        assertFalse(near(4.6, 6.0, 0, 0));
    }

    @Test
    void redTrenchesAreTheMirror() {
        double x = FieldConstants.kFieldLength - 4.6;
        assertTrue(near(x, FieldConstants.kFieldWidth - 0.7, 0, 0));
        assertTrue(near(x, 0.7, 0, 0));
        assertFalse(near(x, FieldConstants.kFieldWidth - 2.1, 0, 0));
    }

    @Test
    void openFieldIsNotNear() {
        assertFalse(TrenchGuard.nearTrench(new Pose2d(8, 4, Rotation2d.ZERO), kStill));
        assertFalse(TrenchGuard.nearTrench(new Pose2d(2, 4, Rotation2d.ZERO), kStill));
    }
}
