package frc.robot.subsystems.vision;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Decides if a vision pose is good enough to use, and how much to trust it.
 *
 * <p>The pose estimator mixes wheel odometry and vision. For each vision pose it needs a standard
 * deviation: a small value says "trust this a lot", a large value says "only nudge the pose". A
 * tag far away is seen with fewer pixels, so its error grows about with distance squared. More
 * tags in one solve give a better pose, so the error drops with the tag count squared (as in
 * 6328's 2026 code).
 *
 * <p>These values only mean something next to the odometry standard deviations
 * ({@code DriveConstants.kOdometryStdDevs}): each frame moves the pose by the fraction
 * q / (q + sqrt(q·r)) of its difference, with q and r the odometry and vision variances.
 *
 * <p>This class has no hardware in it, so the unit tests can check it.
 */
public final class VisionFilter {
    private VisionFilter() {}

    /** Single-tag poses with more ambiguity than this can be the "flipped" solution. */
    public static final double kMaxAmbiguity = 0.2;
    /** One tag farther than this gives a pose that is too noisy to use. */
    public static final double kMaxSingleTagDistance = 4.0;
    /**
     * A single-tag pose this far from the gyro heading, radians, is a wrong solution: neither of
     * the two solutions of the tag agrees with the gyro.
     */
    public static final double kMaxSingleTagHeadingError = Math.toRadians(15);
    /** The robot is on the floor. A pose higher or lower than this is a bad solve. */
    public static final double kMaxZError = 0.25;
    /** Standard deviation per (meter of tag distance) squared, for x and y, in meters. */
    public static final double kXyCoefficient = 0.01;
    /** Standard deviation per (meter of tag distance) squared, for heading, in radians. */
    public static final double kThetaCoefficient = 0.03;
    /** No pose is better than this, however near the tags are (camera noise, latency). */
    public static final double kMinXyStdDev = 0.02;
    public static final double kMinThetaStdDev = 0.02;
    /**
     * Multi-tag heading only from tags nearer than this, meters: far away, a pixel of corner
     * error turns the solved heading by too much.
     *
     * <p>How to tune the coefficients: park the robot, log "Vision/Accepted Poses" for 30 s at
     * 1, 3 and 5 m from the tags, and set kXyCoefficient so that the standard deviation it gives
     * matches the spread of the logged poses.
     */
    public static final double kMaxHeadingDistance = 4.0;
    /** "Do not use the heading": a single tag cannot measure it well, the gyro is far better. */
    public static final double kUntrusted = 1e6;

    /**
     * One pose from one camera frame.
     *
     * @param pose robot pose from the camera
     * @param tagCount tags used in the solve
     * @param averageDistance mean camera-to-tag distance, meters
     * @param ambiguity pose ambiguity of a single-tag solve (0 to 1); ignored for multi-tag
     * @param headingError |gyro heading - pose heading|, radians; ignored for multi-tag (a wrong
     *     gyro, after a Pigeon reboot, must not block the frames that can fix it)
     */
    public record Observation(Pose3d pose, int tagCount, double averageDistance, double ambiguity,
                              double headingError) {}

    /** Accept or reject, and the standard deviations to use when accepted. */
    public record Result(boolean accepted, String reason, double xyStdDev, double thetaStdDev) {
        static Result reject(String reason) {
            return new Result(false, reason, kUntrusted, kUntrusted);
        }
    }

    /**
     * One tag gives two robot poses: the solution with the lower error and the "flipped" one. The
     * lower error can be the wrong one; the gyro tells them apart. Returns the pose whose heading
     * is nearer {@code heading} ({@code best} on a tie).
     */
    public static Pose3d closerToHeading(Pose3d best, Pose3d alt, Rotation2d heading) {
        double bestError = Math.abs(best.toPose2d().getRotation().minus(heading).getRadians());
        double altError = Math.abs(alt.toPose2d().getRotation().minus(heading).getRadians());
        return altError < bestError ? alt : best;
    }

    /**
     * @param trustHeading may vision set the heading at all? Off until the cameras are checked
     *     ({@link VisionConstants#kCamerasChecked}); even then only for 2 or more tags within
     *     {@link #kMaxHeadingDistance}.
     */
    public static Result evaluate(Observation obs, double fieldLength, double fieldWidth, boolean trustHeading) {
        if (obs.tagCount() < 1) {
            return Result.reject("no tags");
        }
        if (Math.abs(obs.pose().getZ()) > kMaxZError) {
            return Result.reject("z");
        }
        double x = obs.pose().getX();
        double y = obs.pose().getY();
        if (x < 0 || x > fieldLength || y < 0 || y > fieldWidth) {
            return Result.reject("off field");
        }
        if (obs.tagCount() == 1) {
            if (obs.ambiguity() > kMaxAmbiguity) {
                return Result.reject("ambiguity");
            }
            if (obs.averageDistance() > kMaxSingleTagDistance) {
                return Result.reject("single tag too far");
            }
            if (obs.headingError() > kMaxSingleTagHeadingError) {
                return Result.reject("heading");
            }
        }

        double distanceSquared = obs.averageDistance() * obs.averageDistance();
        double tagsSquared = obs.tagCount() * obs.tagCount();
        double xy = Math.max(kMinXyStdDev, kXyCoefficient * distanceSquared / tagsSquared);
        boolean useHeading = trustHeading && obs.tagCount() >= 2 && obs.averageDistance() <= kMaxHeadingDistance;
        double theta = useHeading
            ? Math.max(kMinThetaStdDev, kThetaCoefficient * distanceSquared / tagsSquared)
            : kUntrusted;
        return new Result(true, "ok", xy, theta);
    }
}
