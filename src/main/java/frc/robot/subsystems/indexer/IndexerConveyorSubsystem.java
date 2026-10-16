package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.ParadoxTalon;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.simulation.FlywheelSim;

import static java.util.Arrays.asList;
import java.util.List;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class IndexerConveyorSubsystem extends SubsystemBase {
    private final MotionMagicVelocityDutyCycle m_velocityRequest = new MotionMagicVelocityDutyCycle(0);
    private final StaticBrake m_brakeRequest = new StaticBrake();

    private final CANBus k_CANBus = CANIDConstants.k_canBus1;
    private final ParadoxTalon m_indexerConveyorTop = new ParadoxTalon(CANIDConstants.indexerConveyorLeader, k_CANBus, m_brakeRequest, 0.1);
    private final ParadoxTalon m_indexerConveyorBottom = new ParadoxTalon(CANIDConstants.indexerConveyorFollower, k_CANBus, m_brakeRequest, 0.1);

    private final double k_gearRatio = 1;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), IndexerConstants.k_indexerConveyorMOI, k_gearRatio), DCMotor.getKrakenX60(1));

    public IndexerConveyorSubsystem() {
        m_indexerConveyorTop.getConfigurator().apply(IndexerConstants.m_indexerConveyorConfigs);
        m_indexerConveyorBottom.getConfigurator().apply(IndexerConstants.m_indexerConveyorConfigs.withMotorOutput(IndexerConstants.m_indexerConveyorConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)));


        if (Utils.isSimulation()){
            var talonFXSim = m_indexerConveyorTop.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }
    public Command run() {
        return Commands.runEnd(() -> {
            // TODO: Different Speeds
            setSpeed(asList(m_indexerConveyorTop, m_indexerConveyorBottom), asList(IndexerConstants.k_conveyorRPS, IndexerConstants.k_conveyorRPS));
            }, () -> {
                stop(m_indexerConveyorTop, m_indexerConveyorBottom);
            }, this); 
    }

    public Command runPower(){
        return Commands.runEnd(() -> {
            
            m_indexerConveyorBottom.setThrottle(0.9);
            m_indexerConveyorTop.setThrottle(1.0);
        }, () -> {
            m_indexerConveyorTop.setControl(m_brakeRequest);
            m_indexerConveyorBottom.setControl(m_brakeRequest);
        }, this);
    }
    
    public void setSpeed(List<ParadoxTalon> motors, List<Double> speeds){
        for (int i = 0; i < motors.size(); i++) {
            // motors.get(i).setControl(m_velocityRequest.withVelocity(speeds.get(i)));
            motors.get(i).setControl(m_velocityRequest.withVelocity(speeds.get(i)));
        }
    }

        public void stop(ParadoxTalon... motors){
        for (ParadoxTalon motor : motors) {
            motor.setControl(m_brakeRequest);
        }
    }
    
    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_indexerConveyorTop.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(m_indexerConveyorTop.getMotorVoltage().getValue().unit()));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRotorAcceleration(m_motorSimModel.getAngularAcceleration() * k_gearRatio);
        talonFXSim.setRotorVelocity(Units.radiansToRotations(m_motorSimModel.getAngularVelocity()) * k_gearRatio);
            
        //Multipled by 0.159 to convert rad/s to rps
        Telemetry.log("Indexer Conveyor Motor Speed", Units.radiansToRotations(m_motorSimModel.getAngularVelocity()));

    }
}
