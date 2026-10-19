package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.utils.ParadoxTalon;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.simulation.FlywheelSim;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class IndexerRollerSubsystem extends SubsystemBase {
    private final MotionMagicVelocityVoltage m_velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(true);
    private final CoastOut m_coastRequest = new CoastOut();
    private final CANBus k_CANBus = CANIDConstants.k_canBus1;
    private final ParadoxTalon m_indexerRollerMotor = new ParadoxTalon(CANIDConstants.indexerRoller, k_CANBus, m_coastRequest, 0.1);
    

    private final double k_gearRatio = 0.31;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), IndexerConstants.k_indexerRollerMOI, k_gearRatio), DCMotor.getKrakenX60(1));

    // private enum targetDirection {
    //     IN,
    //     OUT,
    //     STILL
    // };
    // private targetDirection m_runningDirection = targetDirection.STILL;
    // public Trigger isJammed = new Trigger(() -> {
    //     if (m_runningDirection == targetDirection.STILL)
    //         return false;
        
    //     return (Math.abs(m_indexerRollerMotor.getVelocity().getValueAsDouble() - (m_runningDirection == targetDirection.IN ? IndexerConstants.k_conveyorInRPS : IndexerConstants.k_conveyorOutRPS)) < IndexerConstants.k_jamThreshold);
    // }).onTrue(Commands.run(() -> {m_indexerRollerMotor.setControl(m_request.withVelocity(m_runningDirection == targetDirection.IN ? IndexerConstants.k_rollerOutRPS : IndexerConstants.k_rollerInRPS));}, this));


    public IndexerRollerSubsystem() {
        m_indexerRollerMotor.getConfigurator().apply(IndexerConstants.m_indexerRollerConfigs);

        if (Utils.isSimulation()){
            var talonFXSim = m_indexerRollerMotor.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }
    public Command run(boolean in) {
        return Commands.runEnd(() -> {   
            // m_runningDirection = in ? targetDirection.IN : targetDirection.OUT;
            m_indexerRollerMotor.setControl(m_velocityRequest.withVelocity(in ? IndexerConstants.k_rollerInRPS : IndexerConstants.k_rollerOutRPS));
        }, () -> {
            m_indexerRollerMotor.setControl(m_coastRequest);
            // m_runningDirection = targetDirection.STILL;
        }, this); }
    
    public Command runPower(boolean in){
        return Commands.runEnd(() -> {
            
            m_indexerRollerMotor.setThrottle(1 * (in ? 1 : -1));
        }, () -> {
            m_indexerRollerMotor.setControl(m_coastRequest);
        }, this);
    }

    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_indexerRollerMotor.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(m_indexerRollerMotor.getMotorVoltage().getValue().unit()));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRotorAcceleration(m_motorSimModel.getAngularAcceleration() * k_gearRatio);
        talonFXSim.setRotorVelocity(Units.radiansToRotations(m_motorSimModel.getAngularVelocity()) * k_gearRatio);
            
        //Multipled by 0.159 to convert rad/s to rps
        Telemetry.log("Indexer Roller Motor Speed", Units.radiansToRotations(m_motorSimModel.getAngularVelocity()));

    }
}
