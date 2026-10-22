package frc.robot.subsystems.drive;

import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

/**
 * Publishes the drive state for AdvantageScope, Elastic and the .wpilog.
 *
 * <ul>
 *   <li>{@code Drive/...}: struct topics (pose, module velocities and targets) for AdvantageScope.
 *   <li>{@code Swerve Drive}: the nine numbers that Elastic's swerve widget reads (it cannot read
 *       structs).
 *   <li>{@code Field}: Elastic's field widget. Its "Robot" must be one Pose2d; WPILib's Field2d
 *       sends an array, which this Elastic version does not draw.
 * </ul>
 */
class DriveTelemetry {
    private static final String[] kCorners = {"Front Left", "Front Right", "Back Left", "Back Right"};

    private final TelemetryTable m_swerve = Telemetry.getTable("Swerve Drive");
    private final TelemetryTable m_field = Telemetry.getTable("Field");

    DriveTelemetry() {
        m_swerve.setType("SwerveDrive");
        m_field.setType("Field2d");
    }

    void update(SwerveDriveState state) {
        Telemetry.log("Drive/Pose", state.Pose);
        Telemetry.log("Drive/Module Velocities", state.ModuleVelocities);
        Telemetry.log("Drive/Module Targets", state.ModuleTargets);
        Telemetry.log("Drive/Module Positions", state.ModulePositions);
        Telemetry.log("Drive/Velocity", state.Velocity);
        Telemetry.log("Drive/Speed mps", Math.hypot(state.Velocity.vx, state.Velocity.vy));
        Telemetry.log("Drive/Raw Heading deg", state.RawHeading.getDegrees());
        Telemetry.log("Drive/Odometry Hz", state.OdometryPeriod > 0 ? 1.0 / state.OdometryPeriod : 0);
        Telemetry.log("Drive/Failed DAQs", state.FailedDaqs);

        for (int i = 0; i < kCorners.length && i < state.ModuleVelocities.length; i++) {
            m_swerve.log(kCorners[i] + " Angle", state.ModuleVelocities[i].angle.getRadians());
            m_swerve.log(kCorners[i] + " Velocity", state.ModuleVelocities[i].velocity);
        }
        m_swerve.log("Robot Angle", state.Pose.getRotation().getRadians());

        m_field.log("Robot", state.Pose);
    }
}
