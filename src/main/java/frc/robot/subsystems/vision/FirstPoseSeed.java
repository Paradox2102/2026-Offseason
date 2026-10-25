package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
 * <p>So, before the first pose: poses (single-tag or multi-tag) from two or more DIFFERENT cameras
 * that agree in place and heading, at about the same time, set the pose. A flipped single-tag solution repeats
 * frame after frame in one camera, but two cameras looking different ways do not flip to the same
 * wrong pose.
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
    /** Cameras that must agree. */
    public static final int kMinCameras = 2;

    /** The mean pose of the largest set of agreeing cameras (at least {@link #kMinCameras}), or empty. */
    public static Optional<Pose2d> find(List<Candidate> candidates) {
        List<Candidate> bestGroup = List.of();
        double bestSpread = Double.POSITIVE_INFINITY;
        for (var anchor : candidates) {
            // For each other camera, its candidate nearest the anchor that agrees with it.
            Map<String, Candidate> nearest = new HashMap<>();
            Map<String, Double> nearestDistance = new HashMap<>();
            for (var other : candidates) {
                if (other.camera().equals(anchor.camera()) || !agree(anchor, other)) {
                    continue;
                }
                double d = distance(anchor, other);
                if (d < nearestDistance.getOrDefault(other.camera(), Double.POSITIVE_INFINITY)) {
                    nearest.put(other.camera(), other);
                    nearestDistance.put(other.camera(), d);
                }
            }
            List<Candidate> group = new ArrayList<>(nearest.values());
            group.add(anchor);
            double spread = nearestDistance.values().stream().mapToDouble(Double::doubleValue).sum();
            if (group.size() > bestGroup.size() || (group.size() == bestGroup.size() && spread < bestSpread)) {
                bestGroup = group;
                bestSpread = spread;
            }
        }
        if (bestGroup.size() < kMinCameras) {
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
