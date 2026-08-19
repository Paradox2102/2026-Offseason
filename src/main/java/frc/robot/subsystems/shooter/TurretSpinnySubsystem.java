package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utils.DoubleNumber;

public class TurretSpinnySubsystem extends SubsystemBase{
    private final TalonFX m_shooterAzimuth = new TalonFX(CANIDConstants.shooterAzimuth, new CANBus("1"));   // TODO - need to assign canbus numbers
    
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public TurretSpinnySubsystem() {
        m_shooterAzimuth.getConfigurator().apply(ShooterConstants.k_turretConfig);
    }

    public Command rotateToHub() {
        return Commands.run(() -> {
            var rotationTriangle = new Translation2d(Subsystems.m_swerveSubsystem.getSotmHubPos().getX() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getX() : FieldConstants.k_blueHub.getX()), (Subsystems.m_swerveSubsystem.getSotmHubPos().getY() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getY() : FieldConstants.k_blueHub.getY())));
            var targetRotation = Math.atan2(rotationTriangle.getY(), rotationTriangle.getX());
            double bestRotation = targetRotation;
            for (int i = -1; i <= 1; i++) {
                bestRotation = checkBestRotation(bestRotation, targetRotation + (i * (2 * Math.PI)));
            }
            Globals.setPosition(m_shooterAzimuth, m_request, bestRotation);       
        }, this);
    }

    private double checkBestRotation(double bestRotation, double testRotation){
        var robotRotation = Subsystems.m_swerveSubsystem.getSotmHubPos().getRotation().getRadians();
        if (DoubleNumber.isBetween(testRotation, ShooterConstants.k_turretBounds)){
            if ((testRotation - robotRotation) < (bestRotation - robotRotation)){
                bestRotation = testRotation;
            }
        }
        
        return bestRotation;
    }

    
}
