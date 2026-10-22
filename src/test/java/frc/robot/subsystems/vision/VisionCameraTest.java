package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

import frc.robot.constants.FieldConstants;

class VisionCameraTest {
    private static final int kTag = 10;
    private static final Transform3d kRobotToCamera =
        new Transform3d(new Translation3d(-0.3, 0.2, 0.4), new Rotation3d(0, Math.toRadians(-25), Math.PI));

    private final VisionCamera m_camera = new VisionCamera(new VisionConstants.CameraConfig("test_cam", kRobotToCamera));

    /** A robot 2 m in front of tag 10 (along the tag's facing direction), on the floor. */
    private static Pose3d robotPose() {
        Pose3d tag = FieldConstants.kField.getTagPose(kTag).orElseThrow();
        var front = tag.toPose2d().transformBy(new org.wpilib.math.geometry.Transform2d(2.0, 0.0, org.wpilib.math.geometry.Rotation2d.ZERO));
        return new Pose3d(front);
    }

    /** A result that sees tag 10 from {@link #robotPose()}, captured at {@code seconds}. */
    private static PhotonPipelineResult resultAt(double seconds, double ambiguity) {
        Pose3d camera = robotPose().transformBy(kRobotToCamera);
        Pose3d tag = FieldConstants.kField.getTagPose(kTag).orElseThrow();
        Transform3d cameraToTag = new Transform3d(camera, tag);
        // The pose comes from cameraToTag; the corners only need to exist (the constructor checks).
        var corners = List.of(new TargetCorner(0, 0), new TargetCorner(10, 0), new TargetCorner(10, 10), new TargetCorner(0, 10));
        var target = new PhotonTrackedTarget(0, 0, 1, 0, kTag, -1, -1f, cameraToTag, cameraToTag, ambiguity, corners, corners);
        long nanos = (long) (seconds * 1e9);
        var metadata = new PhotonPipelineMetadata(nanos, nanos, 1, 0); // capture time, publish time, sequence id, time since pong
        return new PhotonPipelineResult(metadata, List.of(target), Optional.empty());
    }

    @Test
    void emptyResultsGiveNoMeasurements() {
        assertTrue(m_camera.process(List.of()).isEmpty());
    }

    @Test
    void eachResultGivesItsOwnMeasurement() {
        var measurements = m_camera.process(List.of(resultAt(10.0, 0.05), resultAt(10.03, 0.05)));
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
        assertTrue(m_camera.process(List.of(resultAt(10.0, 0.6))).isEmpty());
        assertEquals("ambiguity", m_camera.lastRejectReason());
    }
}
