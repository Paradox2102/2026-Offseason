package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.telemetry.Telemetry;

import frc.robot.constants.FieldConstants;
import frc.robot.health.HealthMonitor;
import frc.robot.subsystems.drive.Drive;

/**
 * Four PhotonVision cameras that correct the drive's pose with AprilTags.
 *
 * <p>Wheel odometry drifts (wheel slip, bumps); AprilTags have known places on the field, so a
 * camera that sees one can say where the robot is. The drive's pose estimator mixes the two: it
 * trusts each vision pose as much as {@link VisionFilter} says.
 *
 * <p>Per camera this logs, under {@code Vision/<name>/}: Connected, Frames, Tag Count, Accepted
 * and Rejected poses, Reject Reason, and the robot roll, pitch and height of the last estimate.
 * The robot sits flat on the floor, so a roll or pitch far from 0 means that camera's angles in
 * {@link VisionConstants} are wrong.
 */
public class Vision extends SubsystemBase {
    private final Drive m_drive;
    private final List<VisionCamera> m_cameras = new ArrayList<>();
    private VisionSystemSim m_sim = null;
    private int m_simResetCount = 0;

    public Vision(Drive drive) {
        m_drive = drive;
        for (var config : VisionConstants.kCameras) {
            var camera = new VisionCamera(config);
            m_cameras.add(camera);
            HealthMonitor.registerCamera(camera.getCamera());
        }
        if (RobotBase.isSimulation()) {
            setupSim();
        }
    }

    @Override
    public void periodic() {
        var driveSim = m_drive.getSim();
        if (m_sim != null && driveSim != null) {
            if (driveSim.resetCount() != m_simResetCount) {
                // The simulated robot jumped: forget the pose history, or the sim makes frames
                // from poses between the old and the new place.
                m_simResetCount = driveSim.resetCount();
                m_sim.resetRobotPose(driveSim.truthPose());
            }
            // The sim cameras look from the true pose, not from the estimate that vision corrects.
            m_sim.update(driveSim.truthPose());
        }

        List<Pose2d> accepted = new ArrayList<>();
        List<Pose2d> rejected = new ArrayList<>();
        List<Pose3d> tags = new ArrayList<>();
        for (var camera : m_cameras) {
            var frames = camera.readNewFrames();
            var measurements = camera.process(frames);
            for (var m : measurements) {
                // Same clock on the SystemCore: the capture time goes in unchanged.
                m_drive.addVisionMeasurement(m.pose(), m.timestamp(), m.stdDevs());
            }
            for (var frame : frames) {
                for (var target : frame.getTargets()) {
                    FieldConstants.kField.getTagPose(target.getFiducialId()).ifPresent(tags::add);
                }
            }
            accepted.addAll(camera.acceptedPoses());
            rejected.addAll(camera.rejectedPoses());
            log(camera, frames.size());
        }

        Telemetry.log("Vision/Accepted Poses", accepted, Pose2d.struct);
        Telemetry.log("Vision/Rejected Poses", rejected, Pose2d.struct);
        Telemetry.log("Vision/Seen Tags", tags, Pose3d.struct);
        Telemetry.log("Vision/Has Target", !accepted.isEmpty());
        var field = Telemetry.getTable("Field");
        field.log("Vision Accepted", accepted, Pose2d.struct);
        field.log("Vision Rejected", rejected, Pose2d.struct);
    }

    private void log(VisionCamera camera, int frameCount) {
        String prefix = "Vision/" + camera.getName() + "/";
        Telemetry.log(prefix + "Connected", camera.getCamera().isConnected());
        Telemetry.log(prefix + "Frames", frameCount);
        Telemetry.log(prefix + "Tag Count", camera.lastTagCount());
        Telemetry.log(prefix + "Accepted Poses", camera.acceptedPoses(), Pose2d.struct);
        Telemetry.log(prefix + "Rejected Poses", camera.rejectedPoses(), Pose2d.struct);
        Telemetry.log(prefix + "Reject Reason", camera.lastRejectReason());
        camera.lastEstimate().ifPresent(p -> {
            Telemetry.log(prefix + "Robot Roll deg", Math.toDegrees(p.getRotation().getX()));
            Telemetry.log(prefix + "Robot Pitch deg", Math.toDegrees(p.getRotation().getY()));
            Telemetry.log(prefix + "Robot Z m", p.getZ());
        });
    }

    private void setupSim() {
        m_sim = new VisionSystemSim("Vision");
        // The same layout everywhere: the sim tags, the sim cameras and the estimator. (A camera
        // sim made without a layout uses the welded field, 2-4 cm off the AndyMark tags.)
        m_sim.addAprilTags(FieldConstants.kField);
        var props = new SimCameraProperties();
        props.setCalibration(VisionConstants.kSimWidth, VisionConstants.kSimHeight,
            Rotation2d.fromDegrees(VisionConstants.kSimDiagonalFovDeg));
        props.setCalibError(0.25, 0.08);
        props.setFPS(VisionConstants.kSimFps);
        props.setAvgLatencyMs(VisionConstants.kSimLatencyMs);
        props.setLatencyStdDevMs(5);
        for (var camera : m_cameras) {
            var cameraSim = new PhotonCameraSim(camera.getCamera(), props, FieldConstants.kField);
            cameraSim.enableRawStream(false); // saves CPU; the processed stream still shows the tags
            m_sim.addCamera(cameraSim, camera.getConfig().robotToCamera());
        }
    }
}
