package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.IndexerConstants;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class IndexerConveyorSubsystem extends SubsystemBase {
    private final CANBus k_CANBus = CANBus.systemcore(3);
    private final TalonFX m_indexerConveyorLeader = new TalonFX(CANIDConstants.indexerConveyorLeader, k_CANBus);
    private final TalonFX m_indexerConveyorFollower = new TalonFX(CANIDConstants.indexerConveyorFollower, k_CANBus);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final double k_gearRatio = 1;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), IndexerConstants.k_indexerConveyorMOI, k_gearRatio), DCMotor.getKrakenX60(1));

    public IndexerConveyorSubsystem() {
        
        m_indexerConveyorFollower.getConfigurator().apply(IndexerConstants.k_indexerConveyorConfigs);
        m_indexerConveyorLeader.getConfigurator().apply(IndexerConstants.k_indexerConveyorConfigs);

        if (Utils.isSimulation()){
            var talonFXSim = m_indexerConveyorLeader.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }
    public Command run(boolean in) {
        return Commands.runEnd(() -> {
            Globals.setFollowerRPS(m_indexerConveyorLeader, m_indexerConveyorFollower, m_request,
             in ? IndexerConstants.k_conveyorInRPS : IndexerConstants.k_conveyorOutRPS, MotorAlignmentValue.Opposed);   
            }, () -> {
                m_indexerConveyorFollower.setControl(new VoltageOut(0));
            }, this); 
    }
    
    
    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_indexerConveyorLeader.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(m_indexerConveyorLeader.getMotorVoltage().getValue().unit()));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRotorAcceleration(m_motorSimModel.getAngularAcceleration() * k_gearRatio);
        talonFXSim.setRotorVelocity(Units.radiansToRotations(m_motorSimModel.getAngularVelocity()) * k_gearRatio);
            
        //Multipled by 0.159 to convert rad/s to rps
        SmartDashboard.putNumber("Indexer Conveyor Motor Speed", Units.radiansToRotations(m_motorSimModel.getAngularVelocity()));

    }
}
