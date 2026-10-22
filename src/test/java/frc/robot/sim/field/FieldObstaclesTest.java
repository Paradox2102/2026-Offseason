package frc.robot.sim.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

class FieldObstaclesTest {
    private static final double kStroke = 0.324;

    private static FieldObstacles.Contact at(double x, double y, double deg, double rackOut) {
        return FieldObstacles.resolve(new Pose2d(x, y, Rotation2d.fromDegrees(deg)), rackOut, kStroke, 0, 0);
    }

    @Test
    void openFloorIsFree() {
        var c = at(8.0, 2.0, 30, 0);
        assertNull(c.obstacle());
        assertEquals(8.0, c.pose().getX(), 1e-12);
    }

    @Test
    void robotStopsAtTheHub() {
        // Driving +x into the blue hub's alliance-side face (x 4.029): the front bumper is at x + 0.445.
        var c = at(3.70, 4.0, 0, 0);
        assertEquals("blue hub", c.obstacle());
        assertEquals(4.029 - RobotOutline.kFrontX, c.pose().getX(), 1e-6);
        assertEquals(4.0, c.pose().getY(), 1e-6);
    }

    @Test
    void redHubIsTheMirror() {
        var c = at(FieldObstacles.kLength - 3.70, FieldObstacles.kWidth - 4.0, 180, 0);
        assertEquals("red hub", c.obstacle());
    }

    @Test
    void robotStaysInsideTheWalls() {
        var c = at(0.2, 0.2, 45, 0);
        assertEquals("wall", c.obstacle());
        // Turned 45 degrees, the corner of the bumper reaches farthest: push until it touches.
        double reach = (RobotOutline.kFrontX * Math.cos(Math.PI / 4) + RobotOutline.kHalfWidth * Math.sin(Math.PI / 4));
        assertTrue(c.pose().getX() >= reach - 0.03 && c.pose().getY() >= reach - 0.06);
    }

    @Test
    void robotWithTheIntakeOutFitsUnderTheTrench() {
        var c = at(4.6, 0.7, 0, kStroke);
        assertNull(c.obstacle());
        assertTrue(FieldObstacles.trenchHeadroom(new Pose2d(4.6, 0.7, Rotation2d.ZERO), kStroke, kStroke, 0, 0) > 0);
    }

    @Test
    void robotWithTheIntakeInHitsTheTrenchArm() {
        // Driving +x into the arm (x 4.435) with the intake in: 0.67 m tall, the arm is at 0.565.
        var c = at(4.10, 0.7, 0, 0);
        assertEquals("blue trench arm (low y)", c.obstacle());
        assertEquals(4.435 - RobotOutline.kFrontX, c.pose().getX(), 1e-6);
        assertTrue(FieldObstacles.trenchHeadroom(new Pose2d(4.10, 0.7, Rotation2d.ZERO), 0, kStroke, 0, 0) < 0);
    }

    @Test
    void trenchWallKeepsTheRobotInItsLane() {
        // Robot centered at y 1.0 in the trench: its left bumper (y + 0.441) would cross the wall at 1.279.
        var c = at(4.6, 1.0, 0, kStroke);
        assertEquals("blue trench wall (low y)", c.obstacle());
        assertEquals(1.279 - RobotOutline.kHalfWidth, c.pose().getY(), 1e-6);
    }

    @Test
    void extendedIntakeHitsToo() {
        // The body clears the hub by 5 cm, but the intake reaches 0.18 m farther.
        double x = 4.029 - RobotOutline.kFrontX - 0.05;
        var c = at(x, 4.0, 0, kStroke);
        assertNotNull(c.obstacle());
        assertEquals(4.029 - RobotOutline.intakeFront(kStroke), c.pose().getX(), 1e-6);
    }

    @Test
    void notUnderAnArmMeansNoHeadroomLimit() {
        assertEquals(Double.POSITIVE_INFINITY,
            FieldObstacles.trenchHeadroom(new Pose2d(8, 4, Rotation2d.ZERO), 0, kStroke, 0, 0));
    }

    @Test
    void robotWithTheHoodUpHitsTheTrenchArm() {
        var c = FieldObstacles.resolve(new Pose2d(4.10, 0.7, Rotation2d.ZERO), kStroke, kStroke, 20, 0);
        assertEquals("blue trench arm (low y)", c.obstacle());
        assertNull(FieldObstacles.resolve(new Pose2d(4.10, 0.7, Rotation2d.ZERO), kStroke, kStroke, 0, 0).obstacle(),
            "hood down and intake out: it fits");
    }
}
