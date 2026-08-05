package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class HoodSubsystem extends SubsystemBase{

    private final TalonFX m_hoodMotor = new TalonFX(CANIDConstants.hood);

    public HoodSubsystem() {

    }

    
}
