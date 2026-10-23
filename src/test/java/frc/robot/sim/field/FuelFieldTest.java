package frc.robot.sim.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

import frc.robot.sim.field.FuelField.RobotInput;

class FuelFieldTest {
    private static final double kLength = 16.541;
    private static final double kWidth = 8.069;
    private static final double kStroke = 0.324;

    private static FuelField empty(int preload) {
        return new FuelField(kLength, kWidth, List.of(), preload);
    }

    /** A robot far from every ball (robot input null = no robot). */
    private static void run(FuelField field, double seconds, RobotInput robot) {
        for (double t = 0; t < seconds; t += 0.02) {
            field.step(0.02, robot);
        }
    }

    private static RobotInput still(double x, double y, double rackOut, boolean rollers) {
        return new RobotInput(new Pose2d(x, y, Rotation2d.ZERO), 0, 0, 0, rackOut, kStroke, rollers);
    }

    private static RobotInput driving(double x, double y, double vx, double rackOut, boolean rollers) {
        return new RobotInput(new Pose2d(x, y, Rotation2d.ZERO), vx, 0, 0, rackOut, kStroke, rollers);
    }

    @Test
    void startsWithTheStagedBallsAndThePreload() {
        var field = new FuelField(kLength, kWidth, List.of(new Translation2d(8, 4), new Translation2d(8, 5)), 8);
        assertEquals(2, field.ballCount());
        assertEquals(8, field.hopper());
    }

    @Test
    void stillBallStaysPut() {
        var field = empty(0);
        field.addFloorBall(8, 2);
        run(field, 2, null);
        var p = field.positions()[0];
        assertEquals(8, p.getX(), 1e-9);
        assertEquals(FuelField.kRadius, p.getZ(), 1e-9);
    }

    @Test
    void ballRollsDownTheRamp() {
        var field = empty(0);
        field.addFloorBall(4.4, 2.0); // blue bump, on the alliance-zone ramp
        run(field, 3, null);
        assertTrue(field.positions()[0].getX() < 4.0, "rolled off the ramp toward the alliance wall");
    }

    @Test
    void bumpersPushABall() {
        var field = empty(0);
        field.addFloorBall(9.0, 4.0);
        // Robot drives +x at 2 m/s into the ball, intake in.
        for (double x = 8.0; x < 8.8; x += 0.04) {
            field.step(0.02, driving(x, 4.0, 2.0, 0, false));
        }
        var p = field.positions()[0];
        assertTrue(p.getX() > 8.8 + RobotOutline.kFrontX, "the ball stays in front of the bumper");
    }

    @Test
    void runningIntakePicksUpABall() {
        var field = empty(0);
        field.addFloorBall(8.5, 4.0);
        field.step(0.02, still(8.0, 4.0, kStroke, true));
        assertEquals(0, field.ballCount());
        assertEquals(1, field.hopper());
        assertEquals(1, field.pickedUp());
    }

    @Test
    void intakeOutWithRollersStoppedPushes() {
        var field = empty(0);
        field.addFloorBall(8.5, 4.0);
        field.step(0.02, still(8.0, 4.0, kStroke, false));
        assertEquals(1, field.ballCount());
        assertEquals(0, field.hopper());
        assertTrue(field.positions()[0].getX() > 8.0 + RobotOutline.intakeFront(kStroke));
    }

    @Test
    void intakeInDoesNotCollect() {
        var field = empty(0);
        field.addFloorBall(8.45, 4.0);
        field.step(0.02, still(8.0, 4.0, 0, true));
        assertEquals(0, field.hopper());
    }

    @Test
    void hopperStopsAtItsCapacity() {
        var field = empty(FuelField.kHopperCapacity);
        field.addFloorBall(8.5, 4.0);
        field.step(0.02, still(8.0, 4.0, kStroke, true));
        assertEquals(FuelField.kHopperCapacity, field.hopper());
        assertEquals(1, field.ballCount());
    }

    @Test
    void ballDroppedIntoTheHubScoresAndComesOutOnTheNeutralSide() {
        var field = empty(1);
        field.launch(new Translation3d(FuelField.kHubBlueX, FuelField.kHubY, 2.5), 0, 0, 0);
        assertEquals(0, field.hopper());
        run(field, 2, null);
        assertEquals(1, field.scored(false));
        assertEquals(0, field.scored(true));
        var p = field.positions()[0];
        assertTrue(p.getX() > FuelField.kHubBlueX + FuelField.kHubHalfSize, "out on the neutral-zone side");
        assertEquals(0, field.airCount());
    }

    @Test
    void redHubScoresForRed() {
        var field = empty(1);
        field.launch(new Translation3d(kLength - FuelField.kHubBlueX, kWidth - FuelField.kHubY, 2.5), 0, 0, 0);
        run(field, 2, null);
        assertEquals(1, field.scored(true));
        assertTrue(field.positions()[0].getX() < kLength - FuelField.kHubBlueX - FuelField.kHubHalfSize);
    }

    @Test
    void lobFromThreeMetersFlies() {
        // From 3 m short of the hub, 0.46 m up: a shot at 45 degrees lands, misses or scores, but
        // ends on the floor, in the field.
        var field = empty(1);
        double v = 7.0;
        field.launch(new Translation3d(FuelField.kHubBlueX - 3, FuelField.kHubY, 0.46),
            v * Math.cos(Math.PI / 4), 0, v * Math.sin(Math.PI / 4));
        run(field, 5, null);
        assertEquals(0, field.airCount());
        var p = field.positions()[0];
        assertTrue(p.getX() > 0 && p.getX() < kLength && p.getY() > 0 && p.getY() < kWidth);
    }

    @Test
    void aBallThatFallsThroughTheOpeningIsRecordedAsAHit() {
        var field = empty(1);
        field.launch(new Translation3d(FuelField.kHubBlueX + 0.1, FuelField.kHubY - 0.2, 2.5), 0, 0, 0);
        run(field, 2, null);
        var c = field.crossings();
        assertEquals(1, c.size());
        assertTrue(c.get(0).scored());
        assertFalse(c.get(0).red());
        assertEquals(0.1, c.get(0).dx(), 0.01);
        assertEquals(-0.2, c.get(0).dy(), 0.01);
        // Falling 0.67 m from rest: about 0.37 s.
        assertEquals(Math.sqrt(2 * 0.67 / 9.81), c.get(0).flightTime(), 0.03);
    }

    @Test
    void aBallThatFallsBesideTheOpeningIsRecordedAsAMiss() {
        var field = empty(1);
        field.launch(new Translation3d(FuelField.kHubBlueX - 0.8, FuelField.kHubY, 2.5), 0, 0, 0);
        run(field, 2, null);
        var c = field.crossings();
        assertEquals(1, c.size());
        assertFalse(c.get(0).scored());
        assertEquals(-0.8, c.get(0).dx(), 0.01);
        field.clearCrossings();
        assertTrue(field.crossings().isEmpty());
    }

    @Test
    void aBallFarFromEveryHubIsNotRecorded() {
        var field = empty(1);
        field.launch(new Translation3d(8, 2, 2.5), 0, 0, 0);
        run(field, 2, null);
        assertTrue(field.crossings().isEmpty());
    }

    @Test
    void fillHopperSetsTheCount() {
        var field = empty(0);
        field.fillHopper(30);
        assertEquals(30, field.hopper());
        field.fillHopper(99);
        assertEquals(FuelField.kHopperCapacity, field.hopper());
    }

    @Test
    void missLandsOnTheFloor() {
        var field = empty(1);
        field.launch(new Translation3d(8, 2, 1.0), 2, 0, 3);
        run(field, 4, null);
        assertEquals(0, field.scored(false) + field.scored(true));
        assertEquals(0, field.airCount());
        assertEquals(FuelField.kRadius, field.positions()[0].getZ(), 1e-9);
    }

    @Test
    void emptyHopperDoesNotShoot() {
        var field = empty(0);
        assertTrue(!field.launch(new Translation3d(8, 2, 1), 1, 0, 1));
        assertEquals(0, field.shots());
    }

    @Test
    void ballsNeverLeaveTheField() {
        var field = empty(0);
        field.addFloorBall(0.3, 0.3);
        for (double x = 1.5; x > 0.6; x -= 0.05) {
            field.step(0.02, new RobotInput(new Pose2d(x, 0.6, Rotation2d.k180deg), -2.5, 0, 0, 0, kStroke, false));
        }
        var p = field.positions()[0];
        assertTrue(p.getX() >= FuelField.kRadius - 1e-9 && p.getY() >= FuelField.kRadius - 1e-9);
    }

    @Test
    void touchingBallsSeparate() {
        var field = empty(0);
        field.addFloorBall(8.0, 4.0);
        field.addFloorBall(8.05, 4.0);
        run(field, 0.1, null);
        var p = field.positions();
        assertTrue(p[0].getDistance(p[1]) >= 2 * FuelField.kRadius - 1e-6);
    }
}
