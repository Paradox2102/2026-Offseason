package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IntakeConstants.IndexerConstants;

public class IndexerRollerSubsystem extends SubsystemBase {
    private final TalonFX m_intakeRollerMotor = new TalonFX(CANIDConstants.indexerRoller);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public IndexerRollerSubsystem() {
        m_intakeRollerMotor.getConfigurator().apply(new MotorOutputConfigs());
    }
    public Command run(boolean in) {
        return Commands.run(() -> {               
            m_intakeRollerMotor.setControl(m_request.withFeedForward(in ? IndexerConstants.k_rollerInRPM : IndexerConstants.k_rollerOutRPM));
        }, this); }
        
}
