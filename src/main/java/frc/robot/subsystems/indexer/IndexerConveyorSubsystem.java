package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IndexerConstants;

public class IndexerConveyorSubsystem extends SubsystemBase {
    private final TalonFX k_indexerConveyorLeader = new TalonFX(CANIDConstants.indexerConveyorLeader);
    private final TalonFX k_indexerConveyorFollower = new TalonFX(CANIDConstants.indexerConveyorFollower);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);


    public IndexerConveyorSubsystem() {
        k_indexerConveyorFollower.getConfigurator().apply(IndexerConstants.k_indexerConveyorConfigs);
        k_indexerConveyorLeader.getConfigurator().apply(IndexerConstants.k_indexerConveyorConfigs);
    }
    public Command run(boolean in) {
        return Commands.run(() -> {
            Globals.setFollowerRPM(k_indexerConveyorLeader, k_indexerConveyorFollower, m_request,
             in ? IndexerConstants.k_conveyorInRPM : IndexerConstants.k_conveyorOutRPM, MotorAlignmentValue.Opposed);   
        }, this); }
}
