package frc.robot.subsystems.shooter;


import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation2d;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utils.DoubleNumber;
import frc.robot.utils.Constants.CANIDConstants;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.Globals;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.Constants.Subsystems;

public class TurretAimSubsystem extends SubsystemBase{
    private final TalonFX m_shooterAzimuth = new TalonFX(CANIDConstants.shooterAzimuth, null);
    
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public final Trigger m_isOnTarget = new Trigger(() -> (Math.abs(m_request.Position - getTargetRotation()) < 0.5));

    public TurretAimSubsystem() {
        m_shooterAzimuth.getConfigurator().apply(ShooterConstants.k_turretConfig);
    }

    public Command rotateToTarget() {
        return Commands.run(() -> {
            
            Globals.setPosition(m_shooterAzimuth, m_request, getTargetRotation());       
        }, this);
    }

    public Command staticAim() {
        return Commands.run(() -> Globals.setPosition(m_shooterAzimuth, m_request, 0), this);
    }

    private double getTargetRotation(){
        Pose3d hubPose = Subsystems.m_swerveSubsystem.getSotmHubPos();
        var rotationTriangle = new Translation2d(hubPose.getX() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getX() : FieldConstants.k_blueHub.getX()), (hubPose.getY() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getY() : FieldConstants.k_blueHub.getY())));
        var targetRotation = Math.atan2(rotationTriangle.getY(), rotationTriangle.getX());
        double bestRotation = targetRotation;
        for (int i = -1; i <= 1; i++) {
            bestRotation = checkBestRotation(bestRotation, targetRotation + (i * (2 * Math.PI)), hubPose.toPose2d());
        }
        return bestRotation;
    }

    private double checkBestRotation(double bestRotation, double testRotation, Pose2d hubPose){
        var robotRotation = hubPose.getRotation().getRadians();
        if (DoubleNumber.isBetween(testRotation, ShooterConstants.k_turretBounds)){
            if ((testRotation - robotRotation) < (bestRotation - robotRotation)){
                bestRotation = testRotation;
            }
        }
        
        return bestRotation;
    }

    
}
