package frc.robot.autos;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.system.Timer;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.Constants.Subsystems;
import frc.robot.subsystems.drive.SwerveSubsystem;

@Autonomous(name = "Drive Forward Auto")
public class DriveForward extends PeriodicOpMode {
    
    private final Timer m_timer = new Timer();
    private final double k_autoTime = 16.0;
    private final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelVelocities(true)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    @Override
    public void start() {
        Subsystems.m_swerveSubsystem.resetPose(new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(SwerveSubsystem.isRedAlliance() ? 0 : 180))));
        m_timer.start();
    }

    @Override
    public void periodic() {
        if (m_timer.get() > k_autoTime){
            Subsystems.m_swerveSubsystem.setControl(drive.withVelocityX(-2.5).withVelocityY(0).withRotationalRate(0));
        }
    }

    
}
