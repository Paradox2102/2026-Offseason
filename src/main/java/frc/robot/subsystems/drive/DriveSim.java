package frc.robot.subsystems.drive;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.telemetry.Telemetry;

/**
 * Simulator only: where the simulated robot "really" is.
 *
 * <p>The vision sim must see the tags from the true robot pose. The drive's own pose cannot be
 * the truth, because vision changes it: then vision would only ever confirm itself, and the sim
 * could never show vision fixing a wrong pose. The CTRE sim has no wheel slip, so odometry alone
 * (wheels + raw gyro, no vision) is the truth.
 *
 * <p>To see vision work: run "Sim/Kick Pose" from the Tuning tab. It moves the drive's pose 0.5 m
 * and 10 degrees off. Vision should pull "Drive/Pose" back to "Sim/Truth Pose" in a few seconds.
 */
public class DriveSim {
    private final Drive m_drive;
    private final SwerveDriveOdometry m_truth;
    private int m_resetCount = 0;

    DriveSim(Drive drive) {
        m_drive = drive;
        var state = drive.getState();
        m_truth = new SwerveDriveOdometry(drive.getKinematics(), state.RawHeading, state.ModulePositions, state.Pose);
    }

    void periodic() {
        var state = m_drive.getState();
        m_truth.update(state.RawHeading, state.ModulePositions);
        Telemetry.log("Sim/Truth Pose", truthPose());
    }

    public Pose2d truthPose() {
        return m_truth.getPose();
    }

    /** An intentional reset (auto start, driver button) moves the simulated robot too. */
    void resetTruth(Pose2d pose) {
        var state = m_drive.getState();
        m_truth.resetPosition(state.RawHeading, state.ModulePositions, pose);
        m_resetCount++;
    }

    /**
     * Goes up by one each time the simulated robot is moved by a reset. The vision sim watches it:
     * it keeps a history of robot poses to make each frame at its capture time, and after a jump it
     * would make frames from poses between the old and new place.
     */
    public int resetCount() {
        return m_resetCount;
    }

    /** Put the drive's pose (not the truth) 0.5 m and 10 degrees off, to test vision. */
    public void kickPose() {
        Pose2d kicked = m_drive.getPose().transformBy(new Transform2d(0.5, 0.3, Rotation2d.fromDegrees(10)));
        m_drive.resetPoseEstimateOnly(kicked);
    }
}
