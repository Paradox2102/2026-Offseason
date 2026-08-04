package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.IntakeConstants;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final TalonFX m_intakeRollerMotor = new TalonFX(CANIDConstants.intakeRoller);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    public IntakeRollerSubsystem() {
        m_intakeRollerMotor.getConfigurator().apply(IntakeConstants.k_intakeRollerConfigs);
        
    }

    public Command run(boolean in){
        return Commands.run(() -> {
            Globals.setRPM(m_intakeRollerMotor, m_request, in ? IntakeConstants.k_inRPM : IntakeConstants.k_outRPM);
        }, this);
    }
    
}
