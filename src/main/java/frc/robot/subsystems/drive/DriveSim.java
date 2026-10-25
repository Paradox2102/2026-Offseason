package frc.robot.subsystems.drive;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import java.util.function.DoubleSupplier;

import org.wpilib.telemetry.Telemetry;

import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.sim.field.BumpTerrain;
import frc.robot.sim.field.FieldObstacles;

/**
 * Simulator only: where the simulated robot "really" is.
 *
 * <p>The vision sim must see the tags from the true robot pose. The drive's own pose cannot be
 * the truth, because vision changes it: then vision would only ever confirm itself, and the sim
 * could never show vision fixing a wrong pose. The CTRE sim has no wheel slip, so odometry alone
 * (wheels + raw gyro, no vision) is the truth on the flat floor.
 *
 * <p>On a bump ramp the robot covers less ground than its wheels turn (the slope, and a little
 * slip: {@link BumpTerrain#groundFactor}). The truth moves that much less, so the drive's odometry
 * runs ahead of it there and vision must pull it back, as on the real robot.
 *
 * <p>The truth also stops at the field structures ({@link FieldObstacles}): the hub, the trench
 * walls, the tower, the field walls, and a trench arm when the intake is not out. The wheels keep
 * turning against them, so odometry runs on, and vision must correct it there too.
 *
 * <p>To see vision work: run "Sim/Kick Pose" from the Tuning tab. It moves the drive's pose 0.5 m
 * and 10 degrees off. Vision should pull "Drive/Pose" back to "Sim/Truth Pose" in a few seconds.
 */
public class DriveSim {
    private final Drive m_drive;
    /** Wheels and raw gyro only: where the robot would be if every wheel turn moved it. */
    private final SwerveDriveOdometry m_wheels;
    private Pose2d m_lastWheels;
    private Pose2d m_truth;
    private int m_resetCount = 0;
    private DoubleSupplier m_rackOut = () -> 0;
    private DoubleSupplier m_hoodDeg = () -> 0;
    private int m_contacts = 0;
    private String m_lastContact = "none";

    DriveSim(Drive drive) {
        m_drive = drive;
        var state = drive.getState();
        m_wheels = new SwerveDriveOdometry(drive.getKinematics(), state.RawHeading, state.ModulePositions, state.Pose);
        m_lastWheels = m_wheels.getPose();
        m_truth = m_lastWheels;
    }

    void periodic() {
        var state = m_drive.getState();
        Pose2d wheels = m_wheels.update(state.RawHeading, state.ModulePositions);
        // This loop's move, in the robot's frame, scaled by the floor under the wheels. The
        // heading is the gyro's, so the truth turns exactly as much as the wheel odometry.
        Transform2d move = wheels.minus(m_lastWheels);
        double factor = BumpTerrain.groundFactor(m_truth);
        m_truth = m_truth.plus(new Transform2d(move.getTranslation().times(factor), move.getRotation()));
        m_lastWheels = wheels;
        var contact = FieldObstacles.resolve(m_truth, m_rackOut.getAsDouble(), IntakeConstants.kRackStroke,
            m_hoodDeg.getAsDouble(), truthPose3d().getZ());
        if (contact.obstacle() != null) {
            m_truth = contact.pose();
            m_contacts++;
            m_lastContact = contact.obstacle();
        }
        Telemetry.log("Sim/Truth Pose", truthPose());
        Telemetry.log("Sim/Ground Factor", factor);
        Telemetry.log("Sim/Touching", contact.obstacle() == null ? "" : contact.obstacle());
    }

    /** The intake travel and the hood angle, so the sim knows whether the robot fits under a trench arm. */
    public void setRobotHeight(DoubleSupplier rackOut, DoubleSupplier hoodDeg) {
        m_rackOut = rackOut;
        m_hoodDeg = hoodDeg;
    }

    public double rackOut() {
        return m_rackOut.getAsDouble();
    }

    public double hoodDeg() {
        return m_hoodDeg.getAsDouble();
    }

    /** Loops in which the robot touched a field structure, since the start. */
    public int contacts() {
        return m_contacts;
    }

    public String lastContact() {
        return m_lastContact;
    }

    public Pose2d truthPose() {
        return m_truth;
    }

    /** The true pose with the bump under the wheels: height, pitch and roll. */
    public Pose3d truthPose3d() {
        return BumpTerrain.pose3d(m_truth);
    }

    /** An intentional reset (auto start, driver button) moves the simulated robot too. */
    void resetTruth(Pose2d pose) {
        var state = m_drive.getState();
        m_wheels.resetPosition(state.RawHeading, state.ModulePositions, pose);
        m_lastWheels = pose;
        m_truth = pose;
        m_resetCount++;
    }

    /**
     * Sim test only: put the simulated robot here and tell the drive nothing, like a robot that
     * booted with no field pose (the estimate stays at the corner, heading 0).
     */
    public void placeTruthOnly(Pose2d pose) {
        resetTruth(pose);
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
