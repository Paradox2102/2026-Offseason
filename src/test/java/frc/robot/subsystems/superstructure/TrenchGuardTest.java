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

    /** A robot backing east toward the low-y blue arm (heading 180: its rear bumper leads). */
    private static boolean brake(double x, double vx) {
        return TrenchGuard.mustBrake(new Pose2d(x, 0.65, Rotation2d.k180deg), new ChassisVelocities(vx, 0, 0));
    }

    @Test
    void farFromTheArmNoBrake() {
        assertFalse(brake(2.0, 2.0));
    }

    @Test
    void tooCloseToStopBeforeTheArmBrakes() {
        // Rear bumper 0.406 m behind the center, arm at x 4.435. At 2 m/s the robot needs
        // 2^2 / (2 x 4) = 0.5 m (+0.1) to stop.
        double rear = 0.406;
        assertFalse(brake(4.435 - rear - 0.65, 2.0));
        assertTrue(brake(4.435 - rear - 0.55, 2.0));
        // Still: it may creep up to the margin, not closer.
        assertFalse(brake(4.435 - rear - 0.15, 0.0));
        assertTrue(brake(4.435 - rear - 0.05, 0.0));
    }

    @Test
    void drivingAwayFromTheArmNoBrake() {
        assertFalse(brake(4.435 - 0.406 - 0.2, -2.0));
    }

    @Test
    void alreadyUnderTheArmNoBrake() {
        // Stopping under the arm does not help: drive on out.
        assertFalse(brake(4.6, 2.0));
    }

    @Test
    void outsideTheLaneNoBrake() {
        assertFalse(TrenchGuard.mustBrake(new Pose2d(3.8, 2.5, Rotation2d.k180deg), new ChassisVelocities(2.0, 0, 0)));
    }

    @Test
    void theIntakeLeadsWhenTheRobotFacesTheArm() {
        // Heading 0, driving east: the intake front (0.73 m) leads, so the brake comes earlier.
        var pose = new Pose2d(4.435 - 0.73 - 0.55, 0.65, Rotation2d.ZERO);
        assertTrue(TrenchGuard.mustBrake(pose, new ChassisVelocities(2.0, 0, 0)));
    }

    @Test
    void redArmsBrakeToo() {
        double x = FieldConstants.kFieldLength - (4.435 - 0.406 - 0.3);
        // Red robot backing toward its arm (west), heading 0 in the field: its rear leads.
        assertTrue(TrenchGuard.mustBrake(new Pose2d(x, FieldConstants.kFieldWidth - 0.65, Rotation2d.ZERO),
            new ChassisVelocities(-2.0, 0, 0)));
    }

    @Test
    void openFieldIsNotNear() {
        assertFalse(TrenchGuard.nearTrench(new Pose2d(8, 4, Rotation2d.ZERO), kStill));
        assertFalse(TrenchGuard.nearTrench(new Pose2d(2, 4, Rotation2d.ZERO), kStill));
    }
}
