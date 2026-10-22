package frc.robot.sim.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

class BumpTerrainTest {
    @Test
    void floorIsFlatAwayFromTheBumps() {
        assertEquals(0, BumpTerrain.height(2.0, 2.0), 1e-9);
        assertEquals(0, BumpTerrain.height(4.6255, 4.0), 1e-9, "the hub sits between the bumps");
        assertEquals(1.0, BumpTerrain.groundFactor(new Pose2d(2, 2, Rotation2d.ZERO)), 1e-9);
    }

    @Test
    void bumpPeaksAtItsCenterLine() {
        assertEquals(0.165, BumpTerrain.height(4.6255, 2.0), 1e-9);
        assertEquals(0.165, BumpTerrain.height(11.9155, 6.0), 1e-9);
    }

    @Test
    void rampsAreFifteenDegrees() {
        assertEquals(Math.toRadians(15), Math.atan(Math.abs(BumpTerrain.slopeX(4.3, 2.0))), Math.toRadians(0.5));
        assertTrue(BumpTerrain.slopeX(4.3, 2.0) > 0, "the floor rises toward the center line");
        assertTrue(BumpTerrain.slopeX(4.9, 2.0) < 0);
    }

    @Test
    void redBumpsMirrorTheBlueOnes() {
        double fieldLength = 4.6255 + 11.9155;
        for (double x = 3.9; x < 5.4; x += 0.1) {
            assertEquals(BumpTerrain.height(x, 2.0), BumpTerrain.height(fieldLength - x, 8.069 - 2.0), 1e-9);
        }
    }

    @Test
    void robotClimbingNoseFirstPitchesNoseUp() {
        // Front wheels (x + 0.26) on the ramp, back wheels on the floor.
        var pose = BumpTerrain.pose3d(new Pose2d(4.0 - 0.26 + 0.3, 2.0, Rotation2d.ZERO));
        assertTrue(pose.getRotation().getY() < 0, "negative pitch = nose up");
        assertTrue(pose.getZ() > 0);
        assertEquals(0, pose.getRotation().getX(), 1e-9, "no roll on a straight climb");
    }

    @Test
    void robotAlongTheRampRolls() {
        // Facing +y on the blue ramp: the left wheels (robot +y = field -x) are lower.
        var pose = BumpTerrain.pose3d(new Pose2d(4.3, 2.0, Rotation2d.fromDegrees(90)));
        assertTrue(pose.getRotation().getX() < 0, "left side low = negative roll");
    }

    @Test
    void robotOnARampCoversLessGround() {
        double factor = BumpTerrain.groundFactor(new Pose2d(4.3, 2.0, Rotation2d.ZERO));
        assertEquals(Math.cos(Math.toRadians(15)) * BumpTerrain.kRampGrip, factor, 0.01);
    }
}
