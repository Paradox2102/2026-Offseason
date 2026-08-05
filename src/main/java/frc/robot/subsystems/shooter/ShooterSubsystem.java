package frc.robot.subsystems.shooter;

import javax.naming.ldap.SortResponseControl;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class ShooterSubsystem extends SubsystemBase{
    private final TalonFX k_shooterLeader = new TalonFX(CANIDConstants.shooterLeader);
    private final TalonFX k_shooterFollower = new TalonFX(CANIDConstants.shooterFollower);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public ShooterSubsystem() {
        k_shooterFollower.getConfigurator().apply(new MotorOutputConfigs());
        k_shooterLeader.getConfigurator().apply(new MotorOutputConfigs());

    }
}
