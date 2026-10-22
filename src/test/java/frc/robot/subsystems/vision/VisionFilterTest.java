package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

class VisionFilterTest {
    private static final double kFieldLength = 16.518;
    private static final double kFieldWidth = 8.043;

    private static VisionFilter.Result evaluate(Pose3d pose, int tags, double distance, double ambiguity) {
        return VisionFilter.evaluate(
            new VisionFilter.Observation(pose, tags, distance, ambiguity), kFieldLength, kFieldWidth);
    }

    private static Pose3d pose(double x, double y, double z) {
        return new Pose3d(x, y, z, Rotation3d.ZERO);
    }

    @Test
    void multiTagNearbyIsTrustedMost() {
        var near = evaluate(pose(3, 4, 0), 3, 1.0, 0);
        var far = evaluate(pose(3, 4, 0), 3, 4.0, 0);
        assertTrue(near.accepted());
        assertTrue(far.accepted());
        assertTrue(near.xyStdDev() < far.xyStdDev(), "a nearer tag must be trusted more");
    }

    @Test
    void moreTagsAreTrustedMore() {
        var two = evaluate(pose(3, 4, 0), 2, 2.0, 0);
        var four = evaluate(pose(3, 4, 0), 4, 2.0, 0);
        assertEquals(two.xyStdDev() / 2, four.xyStdDev(), 1e-9);
    }

    @Test
    void singleTagHeadingIsNotTrusted() {
        var single = evaluate(pose(3, 4, 0), 1, 2.0, 0.05);
        assertTrue(single.accepted());
        assertTrue(single.thetaStdDev() >= 1e5, "one tag cannot give a good heading");
    }

    @Test
    void rejectsAmbiguousSingleTag() {
        var result = evaluate(pose(3, 4, 0), 1, 2.0, 0.5);
        assertFalse(result.accepted());
        assertEquals("ambiguity", result.reason());
    }

    @Test
    void rejectsFarSingleTag() {
        var result = evaluate(pose(3, 4, 0), 1, 4.5, 0.05);
        assertFalse(result.accepted());
        assertEquals("single tag too far", result.reason());
    }

    @Test
    void rejectsPoseOffTheField() {
        assertEquals("off field", evaluate(pose(-1, 4, 0), 2, 2.0, 0).reason());
        assertEquals("off field", evaluate(pose(3, 9, 0), 2, 2.0, 0).reason());
    }

    @Test
    void rejectsPoseInTheAir() {
        assertEquals("z", evaluate(pose(3, 4, 0.4), 2, 2.0, 0).reason());
    }

    @Test
    void rejectsNoTags() {
        assertEquals("no tags", evaluate(pose(3, 4, 0), 0, 0, 0).reason());
    }
}
