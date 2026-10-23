package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;

class VisionFilterTest {
    private static final double kFieldLength = 16.518;
    private static final double kFieldWidth = 8.043;

    /** Heading not trusted (cameras not checked). */
    private static VisionFilter.Result evaluate(Pose3d pose, int tags, double distance, double ambiguity) {
        return VisionFilter.evaluate(
            new VisionFilter.Observation(pose, tags, distance, ambiguity, 0), kFieldLength, kFieldWidth, false);
    }

    /** Heading trusted (cameras checked, or the sim). */
    private static VisionFilter.Result evaluateTrusted(Pose3d pose, int tags, double distance, double ambiguity) {
        return VisionFilter.evaluate(
            new VisionFilter.Observation(pose, tags, distance, ambiguity, 0), kFieldLength, kFieldWidth, true);
    }

    /** A frame 2 m from its tags, with the gyro this far (degrees) from the vision heading. */
    private static VisionFilter.Result evaluateWithGyroOff(int tags, double headingErrorDeg) {
        return VisionFilter.evaluate(new VisionFilter.Observation(pose(3, 4, 0), tags, 2.0, 0.05,
            Math.toRadians(headingErrorDeg)), kFieldLength, kFieldWidth, true);
    }

    private static Pose3d pose(double x, double y, double z) {
        return new Pose3d(x, y, z, Rotation3d.ZERO);
    }

    private static Pose3d poseWithYaw(double yawDeg) {
        return new Pose3d(3, 4, 0, new Rotation3d(0, 0, Math.toRadians(yawDeg)));
    }

    @Test
    void closerToHeadingPicksTheMatchingSolution() {
        var best = poseWithYaw(40);
        var alt = poseWithYaw(0);
        assertEquals(alt, VisionFilter.closerToHeading(best, alt, Rotation2d.fromDegrees(2)));
    }

    @Test
    void rejectsSingleTagFarFromGyroHeading() {
        var result = evaluateWithGyroOff(1, 20);
        assertFalse(result.accepted());
        assertEquals("heading", result.reason());
    }

    @Test
    void multiTagIgnoresTheGyroHeading() {
        // After a Pigeon reboot the gyro is far off: multi-tag frames must still get through to fix it.
        assertTrue(evaluateWithGyroOff(3, 30).accepted());
    }

    @Test
    void multiTagNearbyIsTrustedMost() {
        var near = evaluate(pose(3, 4, 0), 3, 1.0, 0);
        var far = evaluate(pose(3, 4, 0), 3, 6.0, 0); // above the minimum with 3 tags
        assertTrue(near.accepted());
        assertTrue(far.accepted());
        assertTrue(near.xyStdDev() < far.xyStdDev(), "a nearer tag must be trusted more");
    }

    @Test
    void moreTagsAreTrustedMore() {
        // 6 m: far enough that neither value is at the minimum. The error drops with n squared.
        var two = evaluate(pose(3, 4, 0), 2, 6.0, 0);
        var four = evaluate(pose(3, 4, 0), 4, 6.0, 0);
        assertEquals(two.xyStdDev() / 4, four.xyStdDev(), 1e-9);
    }

    @Test
    void singleTagAtFourMeters() {
        assertEquals(0.16, evaluate(pose(3, 4, 0), 1, 4.0, 0.05).xyStdDev(), 1e-9);
    }

    @Test
    void singleTagHeadingIsNotTrusted() {
        var single = evaluateTrusted(pose(3, 4, 0), 1, 2.0, 0.05);
        assertTrue(single.accepted());
        assertTrue(single.thetaStdDev() >= 1e5, "one tag cannot give a good heading");
    }

    @Test
    void multiTagHeadingIsUsedWhenTrusted() {
        var result = evaluateTrusted(pose(3, 4, 0), 3, 2.0, 0);
        assertEquals(Math.max(0.02, 0.03 * 4 / 9), result.thetaStdDev(), 1e-9);
    }

    @Test
    void headingIsNotUsedUntilTheCamerasAreChecked() {
        var result = evaluate(pose(3, 4, 0), 3, 2.0, 0);
        assertTrue(result.thetaStdDev() >= 1e5, "heading must not be used until the cameras are checked");
    }

    @Test
    void farMultiTagHeadingIsNotUsed() {
        var result = evaluateTrusted(pose(3, 4, 0), 2, 4.5, 0);
        assertTrue(result.accepted(), "x and y are still used");
        assertTrue(result.thetaStdDev() >= 1e5);
    }

    @Test
    void manyFarTagsGiveAHeading() {
        // The left trench end in the sim: 4 tags at 4.66 m. Better than 2 tags at 3 m (0.068 rad).
        var result = evaluateTrusted(pose(3, 4, 0), 4, 4.66, 0);
        assertEquals(0.03 * 4.66 * 4.66 / 16, result.thetaStdDev(), 1e-9);
    }

    @Test
    void twoTagsAtFourMetersStillGiveAHeading() {
        var result = evaluateTrusted(pose(3, 4, 0), 2, 4.0, 0);
        assertEquals(VisionFilter.kMaxHeadingStdDev, result.thetaStdDev(), 1e-9);
    }

    @Test
    void nearTagsStopAtTheMinimum() {
        assertEquals(VisionFilter.kMinXyStdDev, evaluate(pose(3, 4, 0), 4, 0.5, 0).xyStdDev(), 1e-12);
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
