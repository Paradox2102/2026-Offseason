package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

import frc.robot.constants.FieldConstants;

/**
 * One PhotonVision camera: turns its frames into pose measurements for the drive.
 *
 * <p>Each loop, {@link Vision} reads every frame that arrived since the last loop (a camera at 30
 * fps gives 0, 1 or 2 frames per 20 ms loop) and passes them here. Every frame gives at most one
 * measurement, with its own capture time, so the pose estimator can put it at the right moment in
 * the past. Nothing is kept from one loop to the next, so a frame is never added twice.
 */
public class VisionCamera {
    /** A pose for the drive's estimator. */
    public record Measurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    private final VisionConstants.CameraConfig m_config;
    private final boolean m_trustHeading;
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_estimator;

    // For telemetry: what happened to the frames of the last loop.
    private final List<Pose2d> m_accepted = new ArrayList<>();
    private final List<Pose2d> m_rejected = new ArrayList<>();
    private String m_lastRejectReason = "";
    private Pose3d m_lastEstimate = null;
    private int m_lastTagCount = 0;
    private double m_lastLatency = Double.NaN;

    /** @param trustHeading may this camera's multi-tag frames set the heading (see VisionFilter)? */
    public VisionCamera(VisionConstants.CameraConfig config, boolean trustHeading) {
        m_config = config;
        m_trustHeading = trustHeading;
        m_camera = new PhotonCamera(config.name());
        m_estimator = new PhotonPoseEstimator(FieldConstants.kField, config.robotToCamera());
    }

    public String getName() {
        return m_config.name();
    }

    public PhotonCamera getCamera() {
        return m_camera;
    }

    public VisionConstants.CameraConfig getConfig() {
        return m_config;
    }

    /** Read every new frame from the camera (call once per loop: it empties the camera's queue). */
    public List<PhotonPipelineResult> readNewFrames() {
        return m_camera.getAllUnreadResults();
    }

    /**
     * Turn frames into measurements, oldest first. Rejected frames are left out.
     *
     * @param visionNowSeconds now, on the clock of the frame timestamps (see {@link VisionClock})
     */
    public List<Measurement> process(List<PhotonPipelineResult> results, double visionNowSeconds) {
        m_accepted.clear();
        m_rejected.clear();
        m_lastRejectReason = "";
        m_lastLatency = Double.NaN;
        List<Measurement> measurements = new ArrayList<>();

        var sorted = new ArrayList<>(results);
        sorted.sort(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds));
        for (var result : sorted) {
            double age = visionNowSeconds - result.getTimestampSeconds();
            m_lastLatency = age; // sorted, so the last one is the newest frame
            String ageReason = VisionClock.ageRejectReason(age);
            if (!ageReason.isEmpty()) {
                m_lastRejectReason = ageReason;
                continue;
            }

            // Multi-tag (solved on the coprocessor with every tag in view) is far better; with one
            // tag in view, fall back to that tag's least ambiguous solution.
            Optional<EstimatedRobotPose> estimate = m_estimator.estimateCoprocMultiTagPose(result);
            boolean multiTag = estimate.isPresent();
            if (estimate.isEmpty()) {
                estimate = m_estimator.estimateLowestAmbiguityPose(result);
            }
            if (estimate.isEmpty()) {
                continue;
            }
            var est = estimate.get();
            List<PhotonTrackedTarget> used = est.targetsUsed;
            if (multiTag) {
                // PhotonLib lists every tag in the frame; the coprocessor may have left some out.
                var ids = result.getMultiTagResult().get().fiducialIDsUsed;
                used = used.stream().filter(t -> ids.contains((short) t.getFiducialId())).toList();
            }
            int tagCount = used.size();
            double distance = used.stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average().orElse(0);
            double ambiguity = tagCount == 1 ? used.get(0).getPoseAmbiguity() : 0;

            m_lastEstimate = est.estimatedPose;
            m_lastTagCount = tagCount;
            var verdict = VisionFilter.evaluate(
                new VisionFilter.Observation(est.estimatedPose, tagCount, distance, ambiguity),
                FieldConstants.kFieldLength, FieldConstants.kFieldWidth, m_trustHeading);
            Pose2d pose = est.estimatedPose.toPose2d();
            if (verdict.accepted()) {
                m_accepted.add(pose);
                double factor = m_config.stdDevFactor();
                measurements.add(new Measurement(pose, est.timestampSeconds,
                    VecBuilder.fill(verdict.xyStdDev() * factor, verdict.xyStdDev() * factor, verdict.thetaStdDev() * factor)));
            } else {
                m_rejected.add(pose);
                m_lastRejectReason = verdict.reason();
            }
        }
        return measurements;
    }

    public List<Pose2d> acceptedPoses() {
        return m_accepted;
    }

    public List<Pose2d> rejectedPoses() {
        return m_rejected;
    }

    public String lastRejectReason() {
        return m_lastRejectReason;
    }

    /** The full 3D robot pose of the last estimate (its roll, pitch and z show camera-angle errors). */
    public Optional<Pose3d> lastEstimate() {
        return Optional.ofNullable(m_lastEstimate);
    }

    public int lastTagCount() {
        return m_lastTagCount;
    }

    /** Age of the newest frame of the last {@link #process} call, seconds; NaN if it had none. */
    public double lastLatencySeconds() {
        return m_lastLatency;
    }
}
