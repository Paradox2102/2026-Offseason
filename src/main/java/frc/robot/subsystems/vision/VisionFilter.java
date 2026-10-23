package frc.robot.subsystems.vision;

import org.wpilib.math.geometry.Pose3d;

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
     * Let vision set the heading? Off until the cameras are checked: a camera angle a few
     * degrees off in VisionConstants would pull the robot's heading away from the gyro, and turn
     * field-centric driving with it. Turn on when the Vision tab shows robot roll and pitch near
     * 0 for every camera while the robot sits flat. While off, the heading comes from the gyro,
     * the auto start pose and the Reset Heading button.
     *
     * <p>How to tune the coefficients: park the robot, log "Vision/Accepted Poses" for 30 s at
     * 1, 3 and 5 m from the tags, and set kXyCoefficient so that the standard deviation it gives
     * matches the spread of the logged poses.
     */
    public static final boolean kTrustVisionHeading = false;
    /** "Do not use the heading": a single tag cannot measure it well, the gyro is far better. */
    public static final double kUntrusted = 1e6;

    /**
     * One pose from one camera frame.
     *
     * @param pose robot pose from the camera
     * @param tagCount tags used in the solve
     * @param averageDistance mean camera-to-tag distance, meters
     * @param ambiguity pose ambiguity of a single-tag solve (0 to 1); ignored for multi-tag
     */
    public record Observation(Pose3d pose, int tagCount, double averageDistance, double ambiguity) {}

    /** Accept or reject, and the standard deviations to use when accepted. */
    public record Result(boolean accepted, String reason, double xyStdDev, double thetaStdDev) {
        static Result reject(String reason) {
            return new Result(false, reason, kUntrusted, kUntrusted);
        }
    }

    public static Result evaluate(Observation obs, double fieldLength, double fieldWidth) {
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
        }

        double distanceSquared = obs.averageDistance() * obs.averageDistance();
        double tagsSquared = obs.tagCount() * obs.tagCount();
        double xy = Math.max(kMinXyStdDev, kXyCoefficient * distanceSquared / tagsSquared);
        double theta = obs.tagCount() == 1 || !kTrustVisionHeading
            ? kUntrusted
            : Math.max(kMinThetaStdDev, kThetaCoefficient * distanceSquared / tagsSquared);
        return new Result(true, "ok", xy, theta);
    }
}
