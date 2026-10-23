package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.photonvision.targeting.TargetCorner;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

import frc.robot.constants.FieldConstants;

class VisionCameraTest {
    private static final int kTag = 10;
    private static final Transform3d kRobotToCamera =
        new Transform3d(new Translation3d(-0.3, 0.2, 0.4), new Rotation3d(0, Math.toRadians(-25), Math.PI));

    private final VisionCamera m_camera = camera(1.0, false);

    private static VisionCamera camera(double stdDevFactor, boolean trustHeading) {
        return new VisionCamera(new VisionConstants.CameraConfig("test_cam", kRobotToCamera, stdDevFactor), trustHeading);
    }

    /** A robot 2 m in front of tag 10 (along the tag's facing direction), on the floor. */
    private static Pose3d robotPose() {
        Pose3d tag = FieldConstants.kField.getTagPose(kTag).orElseThrow();
        var front = tag.toPose2d().transformBy(new org.wpilib.math.geometry.Transform2d(2.0, 0.0, org.wpilib.math.geometry.Rotation2d.ZERO));
        return new Pose3d(front);
    }

    private static Pose3d cameraPose() {
        return robotPose().transformBy(kRobotToCamera);
    }

    /** The gyro heading when it agrees with {@link #robotPose()}. */
    private static Rotation2d heading() {
        return robotPose().toPose2d().getRotation();
    }

    /** The other single-tag solution: {@link #robotPose()} 0.3 m to the side and turned 50 degrees. */
    private static Pose3d flipped() {
        return robotPose().transformBy(new Transform3d(new Translation3d(0, 0.3, 0), new Rotation3d(0, 0, Math.toRadians(50))));
    }

    private static Transform3d cameraToTagFrom(Pose3d robot) {
        return new Transform3d(robot.transformBy(kRobotToCamera), FieldConstants.kField.getTagPose(kTag).orElseThrow());
    }

    /** One view of tag 10 whose best and alternate solutions put the robot at these two poses. */
    private static PhotonPipelineResult resultWith(Pose3d bestRobot, Pose3d altRobot, double ambiguity) {
        var corners = List.of(new TargetCorner(0, 0), new TargetCorner(10, 0), new TargetCorner(10, 10), new TargetCorner(0, 10));
        var target = new PhotonTrackedTarget(0, 0, 1, 0, kTag, -1, -1f, cameraToTagFrom(bestRobot), cameraToTagFrom(altRobot),
            ambiguity, corners, corners);
        return new PhotonPipelineResult(metadataAt(10.0), List.of(target), Optional.empty());
    }

    /** Tag {@code id} as seen from {@link #robotPose()}. */
    private static PhotonTrackedTarget target(int id, double ambiguity) {
        Transform3d cameraToTag = new Transform3d(cameraPose(), FieldConstants.kField.getTagPose(id).orElseThrow());
        // The pose comes from cameraToTag; the corners only need to exist (the constructor checks).
        var corners = List.of(new TargetCorner(0, 0), new TargetCorner(10, 0), new TargetCorner(10, 10), new TargetCorner(0, 10));
        return new PhotonTrackedTarget(0, 0, 1, 0, id, -1, -1f, cameraToTag, cameraToTag, ambiguity, corners, corners);
    }

    private static PhotonPipelineMetadata metadataAt(double seconds) {
        long nanos = (long) (seconds * 1e9);
        return new PhotonPipelineMetadata(nanos, nanos, 1, 0); // capture time, publish time, sequence id, time since pong
    }

    /** A result that sees tag 10 from {@link #robotPose()}, captured at {@code seconds}. */
    private static PhotonPipelineResult resultAt(double seconds, double ambiguity) {
        return new PhotonPipelineResult(metadataAt(seconds), List.of(target(kTag, ambiguity)), Optional.empty());
    }

    /** Sees tags 9, 10 and 11; the coprocessor solved with {@code used} only. */
    private static PhotonPipelineResult multiTagAt(double seconds, List<Short> used) {
        var fieldToCamera = new Transform3d(Pose3d.ZERO, cameraPose());
        var multiTag = new MultiTargetPNPResult(new PnpResult(fieldToCamera, 0.1), used);
        var targets = List.of(target(9, 0.05), target(10, 0.05), target(11, 0.05));
        return new PhotonPipelineResult(metadataAt(seconds), targets, Optional.of(multiTag));
    }

    private static double distanceTo(int id) {
        return target(id, 0).getBestCameraToTarget().getTranslation().getNorm();
    }

    @Test
    void emptyResultsGiveNoMeasurements() {
        assertTrue(m_camera.process(List.of(), 10.03, heading()).isEmpty());
    }

    @Test
    void eachResultGivesItsOwnMeasurement() {
        var measurements = m_camera.process(List.of(resultAt(10.0, 0.05), resultAt(10.03, 0.05)), 10.06, heading());
        assertEquals(2, measurements.size());
        assertEquals(10.0, measurements.get(0).timestamp(), 1e-6);
        assertEquals(10.03, measurements.get(1).timestamp(), 1e-6);
        var expected = robotPose().toPose2d();
        assertEquals(expected.getX(), measurements.get(0).pose().getX(), 1e-6);
        assertEquals(expected.getY(), measurements.get(0).pose().getY(), 1e-6);
        assertEquals(expected.getRotation().getDegrees(), measurements.get(0).pose().getRotation().getDegrees(), 1e-6);
    }

    @Test
    void rejectedPoseIsNotReturned() {
        assertTrue(m_camera.process(List.of(resultAt(10.0, 0.6)), 10.03, heading()).isEmpty());
        assertEquals("ambiguity", m_camera.lastRejectReason());
    }

    @Test
    void oldFrameIsRejected() {
        assertTrue(m_camera.process(List.of(resultAt(10.0, 0.05)), 11.0, heading()).isEmpty());
        assertEquals("old", m_camera.lastRejectReason());
    }

    @Test
    void multiTagCountsOnlyTheTagsInTheSolve() {
        var measurements = m_camera.process(List.of(multiTagAt(10.0, List.of((short) 10, (short) 11))), 10.03, heading());
        assertEquals(1, measurements.size());
        assertEquals(2, m_camera.lastTagCount(), "tag 9 was seen but not used in the solve");
        double distance = (distanceTo(10) + distanceTo(11)) / 2;
        var expected = VisionFilter.evaluate(new VisionFilter.Observation(robotPose(), 2, distance, 0, 0),
            FieldConstants.kFieldLength, FieldConstants.kFieldWidth, false);
        assertEquals(expected.xyStdDev(), measurements.get(0).stdDevs().get(0, 0), 1e-9);
    }

    @Test
    void stdDevFactorScalesTheTrust() {
        double one = camera(1.0, false).process(List.of(resultAt(10.0, 0.05)), 10.03, heading()).get(0).stdDevs().get(0, 0);
        double two = camera(2.0, false).process(List.of(resultAt(10.0, 0.05)), 10.03, heading()).get(0).stdDevs().get(0, 0);
        assertEquals(2 * one, two, 1e-9);
    }

    @Test
    void trustedMultiTagGivesAHeading() {
        var measurements = camera(1.0, true).process(List.of(multiTagAt(10.0, List.of((short) 10, (short) 11))), 10.03, heading());
        assertTrue(measurements.get(0).stdDevs().get(2, 0) < 1);
    }

    @Test
    void singleTagUsesTheSolutionThatMatchesTheGyro() {
        // The lower-error ("best") solution is the wrong one here.
        var measurements = m_camera.process(List.of(resultWith(flipped(), robotPose(), 0.1)), 10.03, heading());
        assertEquals(1, measurements.size());
        var expected = robotPose().toPose2d();
        assertEquals(expected.getX(), measurements.get(0).pose().getX(), 1e-6);
        assertEquals(expected.getY(), measurements.get(0).pose().getY(), 1e-6);
        assertEquals(expected.getRotation().getDegrees(), measurements.get(0).pose().getRotation().getDegrees(), 1e-6);
    }

    @Test
    void singleTagFarFromBothSolutionsIsRejected() {
        var gyro = heading().plus(Rotation2d.fromDegrees(90));
        assertTrue(m_camera.process(List.of(resultWith(flipped(), robotPose(), 0.1)), 10.03, gyro).isEmpty());
        assertEquals("heading", m_camera.lastRejectReason());
    }

    @Test
    void latencyIsTheAgeOfTheNewestFrame() {
        m_camera.process(List.of(resultAt(10.0, 0.05), resultAt(10.03, 0.05)), 10.05, heading());
        assertEquals(0.02, m_camera.lastLatencySeconds(), 1e-6);
    }
}
