package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The first field pose, from cameras that agree. No hardware: unit tested.
 *
 * <p>Until the robot has a field pose, its gyro heading is only "where it pointed at boot", not a
 * field heading. Single-tag poses are checked against that heading ({@link VisionFilter}), so with
 * the gyro far off every one of them is rejected; only a multi-tag frame could correct it. On the
 * robot (2026-10-25, practice field) each camera saw one tag at a time, the gyro was 67 degrees
 * off, and no pose was ever accepted.
 *
 * <p>So, before the first pose: poses (single-tag or multi-tag) that agree in place and heading,
 * at about the same time, set the pose: from two or more cameras, or from three frames of one
 * camera (user, 2026-10-27: one camera that sees one tag is enough). The single-tag ambiguity
 * check (VisionFilter) still applies. A flipped solution can repeat in one camera, so two
 * cameras are preferred when both are there.
 */
public final class FirstPoseSeed {
    private FirstPoseSeed() {}

    /** A multi-tag pose, or a single-tag pose that passed every check but the gyro heading; {@code time} on the vision clock. */
    public record Candidate(String camera, Pose2d pose, double time) {}

    /** Poses agree within this distance, meters ... */
    public static final double kMaxDistance = 0.5;
    /** ... and this heading, radians ... */
    public static final double kMaxAngle = Math.toRadians(10);
    /** ... captured within this time of each other, seconds. */
    public static final double kWindow = 0.15;
    /** Cameras that agree are enough ... */
    public static final int kMinCameras = 2;
    /**
     * ... or this many frames of ONE camera that agree (user, 2026-10-27: one camera that sees one
     * tag is enough). Three frames (0.1 s at 30 fps), so one glitch frame cannot set the pose.
     */
    public static final int kMinSingleCameraFrames = 3;

    /**
     * The mean pose of the best set of agreeing candidates, or empty: more cameras first, then
     * more frames, then the tighter set. A set counts with {@link #kMinCameras} cameras, or with
     * {@link #kMinSingleCameraFrames} frames of one camera.
     */
    public static Optional<Pose2d> find(List<Candidate> candidates) {
        List<Candidate> bestGroup = List.of();
        int bestCameras = 0;
        double bestSpread = Double.POSITIVE_INFINITY;
        for (var anchor : candidates) {
            List<Candidate> group = new ArrayList<>();
            double spread = 0;
            for (var other : candidates) {
                if (other == anchor || agree(anchor, other)) {
                    group.add(other);
                    spread += distance(anchor, other);
                }
            }
            int cameras = (int) group.stream().map(Candidate::camera).distinct().count();
            boolean enough = cameras >= kMinCameras || group.size() >= kMinSingleCameraFrames;
            if (!enough) {
                continue;
            }
            boolean better = cameras > bestCameras
                || (cameras == bestCameras && group.size() > bestGroup.size())
                || (cameras == bestCameras && group.size() == bestGroup.size() && spread < bestSpread);
            if (better) {
                bestGroup = group;
                bestCameras = cameras;
                bestSpread = spread;
            }
        }
        if (bestGroup.isEmpty()) {
            return Optional.empty();
        }
        double x = 0, y = 0, sin = 0, cos = 0;
        for (var c : bestGroup) {
            x += c.pose().getX();
            y += c.pose().getY();
            sin += c.pose().getRotation().getSin();
            cos += c.pose().getRotation().getCos();
        }
        int n = bestGroup.size();
        return Optional.of(new Pose2d(x / n, y / n, new Rotation2d(Math.atan2(sin, cos))));
    }

    private static boolean agree(Candidate a, Candidate b) {
        return Math.abs(a.time() - b.time()) <= kWindow
            && distance(a, b) <= kMaxDistance
            && Math.abs(a.pose().getRotation().minus(b.pose().getRotation()).getRadians()) <= kMaxAngle;
    }

    private static double distance(Candidate a, Candidate b) {
        return a.pose().getTranslation().getDistance(b.pose().getTranslation());
    }
}
