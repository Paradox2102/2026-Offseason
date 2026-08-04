package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class IndexerConveyorSubsystem extends SubsystemBase {
    private final TalonFX k_indexerConveyorLeader = new TalonFX(CANIDConstants.indexerConveyorLeader);
    private final TalonFX k_indexerConveyorFollower = new TalonFX(CANIDConstants.indexerConveyorFollower);

    public IndexerConveyorSubsystem() {
        
    }

}
