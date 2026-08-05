package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class TurretSpinnySubsystem extends SubsystemBase{
    private final TalonFX m_shooterAzimuth = new TalonFX(CANIDConstants.shooterAzimuth);
    
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public TurretSpinnySubsystem() {
        m_shooterAzimuth.getConfigurator().apply(ShooterConstants.k_turretConfig);
    }

    public Command rotateToHub() {
        return Commands.run(() -> {
            var rotationTriangle = new Translation2d(Subsystems.m_swerveSubsystem.getPose().getX() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getX() : FieldConstants.k_blueHub.getX()), (Subsystems.m_swerveSubsystem.getPose().getY() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getY() : FieldConstants.k_blueHub.getY())));
            Globals.setPosition(m_shooterAzimuth, m_request, Math.atan2(rotationTriangle.getX(), rotationTriangle.getY()));       
        }, this);
    }
    
}
