package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * SysId for the drive motors in amps, not volts.
 *
 * <p>The drive's closed loop uses TorqueCurrentFOC, so its gains (TunerConstants.driveGains) are
 * in amps. CTRE's SysIdSwerveTranslation drives with volts, and gains measured in volts mean
 * nothing to a current loop. This request pushes every wheel with a set current and points the
 * wheels straight ahead.
 *
 * <p>In the WPILib SysId tool, the "voltage" of this test is the drive motors' TorqueCurrent
 * signal (amps). The results kS, kA and kP are then in amps; kV comes out near 0, because a
 * current makes torque, not speed.
 */
public class SysIdSwerveTranslationTorque implements SwerveRequest {
    private final TorqueCurrentFOC m_driveRequest = new TorqueCurrentFOC(0);
    private final PositionVoltage m_steerRequest = new PositionVoltage(0);
    private double m_amps = 0;

    /** Current for each drive motor, amps. */
    public SysIdSwerveTranslationTorque withAmps(double amps) {
        m_amps = amps;
        return this;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modules) {
        for (var module : modules) {
            module.apply(m_driveRequest.withOutput(m_amps), m_steerRequest.withPosition(0));
        }
        return StatusCode.OK;
    }
}
