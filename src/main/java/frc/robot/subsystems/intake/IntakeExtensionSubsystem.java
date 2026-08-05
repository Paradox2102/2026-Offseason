package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.IntakeConstants;

public class IntakeExtensionSubsystem extends SubsystemBase {
    private final TalonFX m_intakeExtensionLeader = new TalonFX(CANIDConstants.intakeExtensionLeader);
    private final TalonFX m_intakeExtensionFollower = new TalonFX(CANIDConstants.intakeExtensionFollower);

    public Trigger m_intakeRetracted = new Trigger(() -> (m_intakeExtensionLeader.getPosition().getValueAsDouble() < 0));

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public IntakeExtensionSubsystem() {
        m_intakeExtensionFollower.getConfigurator().apply(IntakeConstants.k_intakeRollerConfigs);
        m_intakeExtensionLeader.getConfigurator().apply(IntakeConstants.k_intakeRollerConfigs);

    }

    public Command retractIntake() {
        return Commands.run(() -> {
            Globals.setFollowerPosition(m_intakeExtensionLeader, m_intakeExtensionFollower, m_request, 0, MotorAlignmentValue.Aligned);
        }, this).until(() -> m_intakeRetracted.getAsBoolean());
    }
    public Command extendIntake() {
        return Commands.runOnce(() -> {
            Globals.setFollowerPosition(m_intakeExtensionLeader, m_intakeExtensionFollower, m_request, IntakeConstants.k_outPosition, MotorAlignmentValue.Aligned);
        }, this);
    }
}
