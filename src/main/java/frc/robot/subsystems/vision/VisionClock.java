package frc.robot.subsystems.vision;

import org.wpilib.networktables.NetworkTablesJNI;

/**
 * Two clocks meet in vision: PhotonLib gives each frame's capture time on the robot's
 * NetworkTables clock, and the Phoenix pose estimator wants its own clock
 * ({@code Utils.getCurrentTimeSeconds()}). Their epochs can differ, and no API converts one to the
 * other, so {@link Vision} reads both clocks once per loop and shifts each capture time by the
 * difference.
 *
 * <p>A frame far in the past or in the future is a clock problem (a coprocessor that just
 * rebooted, time sync not settled yet), not a pose: it is rejected.
 *
 * <p>This class has no hardware in it, so the unit tests can check it.
 */
public final class VisionClock {
    private VisionClock() {}

    /** A frame older than this, seconds, is not used: the robot has moved too far since. */
    public static final double kMaxAge = 0.5;
    /** A frame up to this far in the future, seconds, is clock jitter; more is a clock fault. */
    public static final double kMaxFuture = 0.01;

    /**
     * A capture time on the vision clock, moved to the drive's clock.
     *
     * @param captureSeconds the frame's capture time, vision clock
     * @param driveNowSeconds now, on the drive's clock
     * @param visionNowSeconds now, on the vision clock (read at the same moment)
     */
    public static double toDriveTime(double captureSeconds, double driveNowSeconds, double visionNowSeconds) {
        return captureSeconds + driveNowSeconds - visionNowSeconds;
    }

    /**
     * Now, on the NetworkTables clock (the clock of PhotonLib frame times), seconds. In WPILib
     * 2027 {@code NetworkTablesJNI.now()} is in nanoseconds (it was microseconds before).
     */
    public static double networkTablesNowSeconds() {
        return NetworkTablesJNI.now() * 1e-9;
    }

    /**
     * A coprocessor that last heard a time-sync pong longer ago than this, seconds, has lost time
     * sync: its frame times cannot be trusted. The same limit as PhotonLib's "not connected to the
     * TimeSyncServer" warning. A shorter limit would drop good frames when a few pongs are lost:
     * an offset from a sync seconds ago is still good (clock drift is parts per million). The
     * coprocessor's ping period is not known yet: "Vision/<camera>/Time Since Sync ms" and its max
     * show it on the robot. The sim reports 1 ms.
     */
    public static final double kMaxTimeSinceSync = 5.0;

    /** Why a frame is not used, from its coprocessor's time since the last pong, or "" if it can be. */
    public static String syncRejectReason(long timeSinceLastPongNanos) {
        return timeSinceLastPongNanos * 1e-9 > kMaxTimeSinceSync ? "time sync" : "";
    }

    /** Why a frame of this age is not used, or "" if it can be used. */
    public static String ageRejectReason(double ageSeconds) {
        if (ageSeconds > kMaxAge) {
            return "old";
        }
        if (ageSeconds < -kMaxFuture) {
            return "future";
        }
        return "";
    }
}
