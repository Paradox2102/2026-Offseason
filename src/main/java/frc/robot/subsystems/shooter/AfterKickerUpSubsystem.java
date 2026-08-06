package frc.robot.subsystems.shooter;

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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IntakeConstants.IndexerConstants;

public class AfterKickerUpSubsystem extends SubsystemBase {
    private final TalonFX m_goUpMotor = new TalonFX(CANIDConstants.goUp);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public AfterKickerUpSubsystem() {
        m_goUpMotor.getConfigurator().apply(new MotorOutputConfigs());
    }
    public Command run() {
        return Commands.run(() -> {               
            m_goUpMotor.setControl(m_request.withFeedForward(ShooterConstants.k_preShootRPM));
        }, this); }
        
}