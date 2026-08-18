package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utils.DoubleNumber;
import frc.robot.utils.Constants.CANIDConstants;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.Globals;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.Constants.Subsystems;

public class TurretAimSubsystem extends SubsystemBase{
    private final TalonFX m_shooterAzimuth = new TalonFX(CANIDConstants.shooterAzimuth);
    
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
        var rotationTriangle = new Translation2d(Subsystems.m_swerveSubsystem.getSotmHubPos().getX() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getX() : FieldConstants.k_blueHub.getX()), (Subsystems.m_swerveSubsystem.getSotmHubPos().getY() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getY() : FieldConstants.k_blueHub.getY())));
        var targetRotation = Math.atan2(rotationTriangle.getY(), rotationTriangle.getX());
        double bestRotation = targetRotation;
        for (int i = -1; i <= 1; i++) {
            bestRotation = checkBestRotation(bestRotation, targetRotation + (i * (2 * Math.PI)));
        }
        return bestRotation;
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
