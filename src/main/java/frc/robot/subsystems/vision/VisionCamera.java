package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
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
 *
 * <p>A frame with one tag gives two possible robot poses. This class takes the one whose heading
 * is nearer the gyro heading, not simply the one with the lower error: the lower error is
 * sometimes the "flipped" solution, tens of centimeters off.
 */
public class VisionCamera {
    /** A pose for the drive's estimator. */
    public record Measurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs, boolean multiTag) {}

    private final VisionConstants.CameraConfig m_config;
    private final boolean m_trustHeading;
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_estimator;

    // For telemetry: what happened to the frames of the last loop.
    private final List<Pose2d> m_accepted = new ArrayList<>();
    /** First-pose candidates ({@link FirstPoseSeed}): multi-tag poses, and single-tag poses that pass every check but the gyro heading. */
    private final List<FirstPoseSeed.Candidate> m_seedCandidates = new ArrayList<>();
    private final List<Pose2d> m_rejected = new ArrayList<>();
    private String m_lastRejectReason = "";
    private Pose3d m_lastEstimate = null;
    private int m_lastTagCount = 0;
    private double m_lastLatency = Double.NaN;
    private double m_lastTimeSinceSync = Double.NaN;
    private double m_maxTimeSinceSync = 0;

    /**
     * The dashboard holds the latency and the reject reason this long, seconds: a 30 fps camera
     * sends no frame in many 20 ms loops, and the values would blink.
     */
    public static final double kDisplayHoldSeconds = 0.5;
    private double m_heldLatency = Double.NaN;
    private double m_heldLatencyAt = Double.NEGATIVE_INFINITY;
    private String m_heldReason = "";
    private double m_heldReasonAt = Double.NEGATIVE_INFINITY;

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

    /** One frame's robot pose and what the filter needs to judge it. */
    private record Solve(Pose3d pose, List<PhotonTrackedTarget> used, double ambiguity, double headingError,
                         boolean multiTag, double distance) {}

    /** An alternate solution shorter than this, meters, is "none" (PhotonVision sent a zero). */
    private static final double kNoAlternate = 1e-6;

    /**
     * Turn frames into measurements, oldest first. Rejected frames are left out.
     *
     * @param visionNowSeconds now, on the clock of the frame timestamps (see {@link VisionClock})
     * @param headingAt the robot heading at a capture time (vision clock): picks the right
     *     single-tag solution. At capture, not now: a fast turn moves the heading tens of degrees
     *     in a frame's latency.
     */
    public List<Measurement> process(List<PhotonPipelineResult> results, double visionNowSeconds,
                                     DoubleFunction<Rotation2d> headingAt) {
        m_accepted.clear();
        m_rejected.clear();
        m_seedCandidates.clear();
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
            // Logged to check the time-sync limit on the robot (see VisionClock.kMaxTimeSinceSync).
            m_lastTimeSinceSync = result.metadata.timeSinceLastPong * 1e-9; // sorted: the last is the newest
            m_maxTimeSinceSync = Math.max(m_maxTimeSinceSync, m_lastTimeSinceSync);
            String syncReason = VisionClock.syncRejectReason(result.metadata.timeSinceLastPong);
            if (!syncReason.isEmpty()) {
                m_lastRejectReason = syncReason;
                continue;
            }

            // Multi-tag (solved on the coprocessor with every tag in view) is far better; with one
            // tag in view, fall back to that tag's solution that agrees with the gyro.
            Optional<Solve> solve = multiTagSolve(result);
            if (solve.isEmpty()) {
                solve = singleTagSolve(result, headingAt.apply(result.getTimestampSeconds()));
                addSeedCandidate(result);
            }
            if (solve.isEmpty()) {
                continue;
            }
            var s = solve.get();
            int tagCount = s.used().size();

            m_lastEstimate = s.pose();
            m_lastTagCount = tagCount;
            var verdict = VisionFilter.evaluate(
                new VisionFilter.Observation(s.pose(), tagCount, s.distance(), s.ambiguity(), s.headingError()),
                FieldConstants.kFieldLength, FieldConstants.kFieldWidth, m_trustHeading);
            Pose2d pose = s.pose().toPose2d();
            if (verdict.accepted() && s.multiTag()) {
                // A multi-tag pose is a first-pose candidate too (FirstPoseSeed): its heading is
                // not trusted to correct the gyro, but poses that agree are.
                m_seedCandidates.add(new FirstPoseSeed.Candidate(m_config.name(), pose, result.getTimestampSeconds()));
            }
            if (verdict.accepted()) {
                m_accepted.add(pose);
                double factor = m_config.stdDevFactor();
                // An untrusted heading stays exactly untrusted, whatever the camera's factor.
                double theta = verdict.thetaStdDev() < VisionFilter.kUntrusted
                    ? verdict.thetaStdDev() * factor : VisionFilter.kUntrusted;
                measurements.add(new Measurement(pose, result.getTimestampSeconds(),
                    VecBuilder.fill(verdict.xyStdDev() * factor, verdict.xyStdDev() * factor, theta),
                    s.multiTag()));
            } else {
                m_rejected.add(pose);
                m_lastRejectReason = verdict.reason();
            }
        }
        if (!results.isEmpty()) {
            m_heldLatency = m_lastLatency;
            m_heldLatencyAt = visionNowSeconds;
        }
        if (!m_lastRejectReason.isEmpty()) {
            m_heldReason = m_lastRejectReason;
            m_heldReasonAt = visionNowSeconds;
        }
        return measurements;
    }

    /**
     * The pose the coprocessor solved from all its tags, if it used 2 or more of the tags in the
     * frame. With fewer, the frame goes to the single-tag path and its gates.
     */
    private Optional<Solve> multiTagSolve(PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> estimate = m_estimator.estimateCoprocMultiTagPose(result);
        if (estimate.isEmpty()) {
            return Optional.empty();
        }
        // PhotonLib lists every tag in the frame; the coprocessor may have left some out.
        var ids = result.getMultiTagResult().get().fiducialIDsUsed;
        var used = estimate.get().targetsUsed.stream().filter(t -> ids.contains((short) t.getFiducialId())).toList();
        if (used.size() < 2) {
            return Optional.empty();
        }
        double distance = used.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average().orElse(0);
        return Optional.of(new Solve(estimate.get().estimatedPose, used, 0, 0, true, distance));
    }

    /**
     * For {@link FirstPoseSeed}: this frame's single tag, PhotonVision's best solution (no gyro to
     * choose with), if it passes every check of {@link VisionFilter} but the gyro heading.
     */
    private void addSeedCandidate(PhotonPipelineResult result) {
        singleTagSolve(result, null).ifPresent(s -> {
            var verdict = VisionFilter.evaluate(
                new VisionFilter.Observation(s.pose(), 1, s.distance(), s.ambiguity(), 0),
                FieldConstants.kFieldLength, FieldConstants.kFieldWidth, m_trustHeading);
            if (verdict.accepted()) {
                m_seedCandidates.add(new FirstPoseSeed.Candidate(m_config.name(), s.pose().toPose2d(), result.getTimestampSeconds()));
            }
        });
    }

    /**
     * The least ambiguous tag of the frame. A tag seen alone has two solutions; the one with the
     * lower error can be the "flipped" one, so take the one whose heading is nearer the gyro
     * (6328 does the same). The filter rejects it if even that one is far from the gyro.
     *
     * @param heading the gyro heading, or null for the best solution with no gyro check
     */
    private Optional<Solve> singleTagSolve(PhotonPipelineResult result, Rotation2d heading) {
        PhotonTrackedTarget target = null;
        for (var t : result.getTargets()) {
            // -1: not a fiducial target.
            if (t.getPoseAmbiguity() != -1 && (target == null || t.getPoseAmbiguity() < target.getPoseAmbiguity())) {
                target = t;
            }
        }
        if (target == null) {
            return Optional.empty();
        }
        var tagPose = FieldConstants.kField.getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) {
            return Optional.empty();
        }
        Transform3d cameraToRobot = m_config.robotToCamera().inverse();
        Transform3d bestToTag = target.getBestCameraToTarget();
        Transform3d altToTag = target.getAlternateCameraToTarget();
        Pose3d best = tagPose.get().transformBy(bestToTag.inverse()).transformBy(cameraToRobot);
        Pose3d pose = best;
        Transform3d picked = bestToTag;
        // A zero alternate would put the robot at the tag: use the best solution alone. No
        // heading (null, for FirstPoseSeed): the best solution, with no gyro check.
        if (heading != null && altToTag.getTranslation().getNorm() >= kNoAlternate) {
            Pose3d alt = tagPose.get().transformBy(altToTag.inverse()).transformBy(cameraToRobot);
            pose = VisionFilter.closerToHeading(best, alt, heading);
            picked = pose == alt ? altToTag : bestToTag;
        }
        double headingError = heading == null ? 0 : Math.abs(pose.toPose2d().getRotation().minus(heading).getRadians());
        return Optional.of(new Solve(pose, List.of(target), target.getPoseAmbiguity(), headingError, false,
            picked.getTranslation().getNorm()));
    }

    public List<Pose2d> acceptedPoses() {
        return m_accepted;
    }

    public List<Pose2d> rejectedPoses() {
        return m_rejected;
    }

    /** This loop's {@link FirstPoseSeed} candidates. */
    public List<FirstPoseSeed.Candidate> seedCandidates() {
        return m_seedCandidates;
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

    /** The newest frame's time since its coprocessor's last time-sync pong, seconds; NaN before any frame. */
    public double lastTimeSinceSyncSeconds() {
        return m_lastTimeSinceSync;
    }

    /** The largest time since a time-sync pong seen on any frame since boot, seconds. */
    public double maxTimeSinceSyncSeconds() {
        return m_maxTimeSinceSync;
    }

    /**
     * For the dashboard: the newest frame's age from the last call that had frames, or NaN once no
     * frame has come for {@link #kDisplayHoldSeconds} (a camera that stopped shows NaN).
     */
    public double displayLatencySeconds(double visionNowSeconds) {
        return visionNowSeconds - m_heldLatencyAt <= kDisplayHoldSeconds ? m_heldLatency : Double.NaN;
    }

    /** For the dashboard: the last reject reason, or "" once it is {@link #kDisplayHoldSeconds} old. */
    public String displayRejectReason(double visionNowSeconds) {
        return visionNowSeconds - m_heldReasonAt <= kDisplayHoldSeconds ? m_heldReason : "";
    }
}
