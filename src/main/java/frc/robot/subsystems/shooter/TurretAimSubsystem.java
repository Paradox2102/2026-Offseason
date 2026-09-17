package frc.robot.subsystems.shooter;


import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utils.DoubleNumber;
// Turret aim gear ratio is 67.86:1
public class TurretAimSubsystem extends SubsystemBase{
    private final CANBus k_CANBus = CANBus.systemcore(1);

    private final TalonFX m_shooterAzimuth = new TalonFX(CANIDConstants.shooterAzimuth, k_CANBus);
    
    public final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

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

    private double getTargetRotation(){ // This should add the turret offset to aiming but I need to check when I get sim up and running.
        Pose3d hubPose = new Pose3d(Subsystems.m_swerveSubsystem.getSotmHubAimPos()).plus(new Transform3d(ShooterConstants.k_turretOffset, new Rotation3d(0, 0, Subsystems.m_swerveSubsystem.getSotmHubAimPos().getRotation().getRadians())));
        var rotationTriangle = new Translation2d(hubPose.getX() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getX() : FieldConstants.k_blueHub.getX()), (hubPose.getY() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getY() : FieldConstants.k_blueHub.getY())));
        var targetRotation = Math.atan2(rotationTriangle.getY(), rotationTriangle.getX());
        double bestRotation = targetRotation;
        for (int i = -1; i <= 1; i++) {
            bestRotation = (checkBestRotation(bestRotation, targetRotation + (i * (2 * Math.PI)), hubPose.toPose2d()) * ShooterConstants.k_shooterGearReduction);
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
