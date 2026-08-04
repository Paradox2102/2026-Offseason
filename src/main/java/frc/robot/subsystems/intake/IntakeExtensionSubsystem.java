package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class IntakeExtensionSubsystem extends SubsystemBase {
    private final TalonFX m_intakeExtensionLeader = new TalonFX(CANIDConstants.intakeExtensionLeader);
    private final TalonFX m_intakeExtensionFollower = new TalonFX(CANIDConstants.intakeExtensionFollower);

    public IntakeExtensionSubsystem() {
        
    }
}
