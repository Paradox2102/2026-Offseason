package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleFunction;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;

import com.ctre.phoenix6.Utils;

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
    private int m_headingMeasurements = 0;
    private int m_multiTagMeasurements = 0;
    private final HeadingReference m_headingReference = new HeadingReference();
    /*
     * A camera at 30 fps sends no frame in many 20 ms loops, so "accepted this loop" blinks.
     * Stay true for 0.2 s after the last accepted pose.
     */
    private final Debouncer m_hasTarget = new Debouncer(0.2, DebounceType.FALLING);

    public Vision(Drive drive) {
        m_drive = drive;
        boolean trustHeading = RobotBase.isSimulation() || VisionConstants.kCamerasChecked;
        for (var config : VisionConstants.kCameras) {
            var camera = new VisionCamera(config, trustHeading);
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
            // The sim cameras look from the true pose, not from the estimate that vision corrects,
            // and in 3D: on a bump the robot tilts, and so do its cameras.
            m_sim.update(driveSim.truthPose3d());
        }

        // Frame times are on the NetworkTables clock (PhotonLib time sync); the Phoenix estimator
        // wants its own clock. Each frame is shifted by the difference (read per camera, below).
        double offsetDriveNow = Utils.getCurrentTimeSeconds();
        Telemetry.log("Vision/Clock Offset ms", (offsetDriveNow - VisionClock.networkTablesNowSeconds()) * 1000);
        Telemetry.log("Vision/Timer Offset ms", (offsetDriveNow - Timer.getTimestamp()) * 1000);

        Rotation2d headingNow = m_drive.getHeading();
        List<Pose2d> accepted = new ArrayList<>();
        List<Pose2d> rejected = new ArrayList<>();
        List<Pose3d> tags = new ArrayList<>();
        for (var camera : m_cameras) {
            var frames = camera.readNewFrames();
            // Read the clocks after the frames: a frame that arrived after an earlier read would
            // look like it came from the future.
            double visionNow = VisionClock.networkTablesNowSeconds();
            double driveNow = Utils.getCurrentTimeSeconds();
            // The estimator's heading when a frame was captured (now, if the buffer has no
            // sample), corrected by recent multi-tag frames when it is far off (HeadingReference).
            DoubleFunction<Rotation2d> estimatorAt = capture -> m_drive
                .samplePoseAt(VisionClock.toDriveTime(capture, driveNow, visionNow))
                .map(Pose2d::getRotation).orElse(headingNow);
            DoubleFunction<Rotation2d> headingAt = capture -> m_headingReference.correct(capture, estimatorAt.apply(capture));
            var measurements = camera.process(frames, visionNow, headingAt);
            for (var m : measurements) {
                if (m.multiTag()) {
                    // Before the estimator takes this frame in, so the offset is the one it corrects.
                    m_headingReference.observe(m.timestamp(), m.pose().getRotation(), estimatorAt.apply(m.timestamp()));
                    m_multiTagMeasurements++;
                }
                m_drive.addVisionMeasurement(m.pose(), VisionClock.toDriveTime(m.timestamp(), driveNow, visionNow),
                    m.stdDevs());
                if (m.stdDevs().get(2, 0) < 1) { // radians: the untrusted value is 1e6
                    m_headingMeasurements++;
                }
            }
            for (var frame : frames) {
                for (var target : frame.getTargets()) {
                    FieldConstants.kField.getTagPose(target.getFiducialId()).ifPresent(tags::add);
                }
            }
            accepted.addAll(camera.acceptedPoses());
            rejected.addAll(camera.rejectedPoses());
            log(camera, frames.size(), visionNow);
        }

        Telemetry.log("Vision/Accepted Poses", accepted, Pose2d.struct);
        Telemetry.log("Vision/Rejected Poses", rejected, Pose2d.struct);
        Telemetry.log("Vision/Seen Tags", tags, Pose3d.struct);
        Telemetry.log("Vision/Has Target", m_hasTarget.calculate(!accepted.isEmpty()));
        var field = Telemetry.getTable("Field");
        field.log("Vision Accepted", accepted, Pose2d.struct);
        field.log("Vision Rejected", rejected, Pose2d.struct);
    }

    /** Measurements sent with a usable heading since boot (the sim smoke test reads it). */
    public int headingMeasurements() {
        return m_headingMeasurements;
    }

    /** Multi-tag measurements sent since boot (the sim smoke test reads it). */
    public int multiTagMeasurements() {
        return m_multiTagMeasurements;
    }

    private void log(VisionCamera camera, int frameCount, double visionNow) {
        String prefix = "Vision/" + camera.getName() + "/";
        Telemetry.log(prefix + "Connected", camera.getCamera().isConnected());
        Telemetry.log(prefix + "Frames", frameCount);
        Telemetry.log(prefix + "Tag Count", camera.lastTagCount());
        Telemetry.log(prefix + "Accepted Poses", camera.acceptedPoses(), Pose2d.struct);
        Telemetry.log(prefix + "Rejected Poses", camera.rejectedPoses(), Pose2d.struct);
        // Held for half a second, so they do not blink; latency is NaN once the camera stops.
        Telemetry.log(prefix + "Reject Reason", camera.displayRejectReason(visionNow));
        Telemetry.log(prefix + "Latency ms", camera.displayLatencySeconds(visionNow) * 1000);
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
