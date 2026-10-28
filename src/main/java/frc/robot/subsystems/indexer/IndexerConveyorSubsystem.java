package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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

public class IndexerConveyorSubsystem extends SubsystemBase {
    private final VelocityVoltage m_velocityRequestTop = new VelocityVoltage(0);
    private final VelocityVoltage m_velocityRequestBottom = m_velocityRequestTop;

    private final StaticBrake m_brakeRequest = new StaticBrake();

    private final CANBus k_CANBus = CANIDConstants.k_canBus1;
    private final ParadoxTalon m_indexerConveyorTop = new ParadoxTalon(CANIDConstants.indexerConveyorLeader, k_CANBus, m_brakeRequest, 0.1);
    private final ParadoxTalon m_indexerConveyorBottom = new ParadoxTalon(CANIDConstants.indexerConveyorFollower, k_CANBus, m_brakeRequest, 0.1);

    private final double k_gearRatio = 1;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX44(1), IndexerConstants.k_indexerConveyorMOI, k_gearRatio), DCMotor.getKrakenX44(1));

    public IndexerConveyorSubsystem() {
        m_indexerConveyorTop.getConfigurator().apply(IndexerConstants.m_indexerConveyorConfigs.withMotorOutput(IndexerConstants.m_indexerConveyorConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)));
        m_indexerConveyorBottom.getConfigurator().apply(IndexerConstants.m_indexerConveyorConfigs);


        if (Utils.isSimulation()){
            var talonFXSim = m_indexerConveyorTop.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }
    public Command run(boolean in) {
        return Commands.runEnd(() -> {
            m_indexerConveyorTop.setControl(m_velocityRequestTop.withVelocity(in ? IndexerConstants.k_conveyorInRPS : IndexerConstants.k_conveyorOutRPS));
            m_indexerConveyorBottom.setControl(m_velocityRequestBottom.withVelocity(in ? IndexerConstants.k_conveyorInRPS : IndexerConstants.k_conveyorOutRPS));    
        }, () -> {
                m_indexerConveyorTop.setControl(m_brakeRequest);
                m_indexerConveyorBottom.setControl(m_brakeRequest);
            }, this); 
    }

        public Command runSlow(boolean in) {
        return Commands.runEnd(() -> {
            m_indexerConveyorTop.setControl(m_velocityRequestTop.withVelocity(in ? IndexerConstants.k_conveyorSlowInRPS : IndexerConstants.k_conveyorSlowOutRPS));
            m_indexerConveyorBottom.setControl(m_velocityRequestBottom.withVelocity(in ? IndexerConstants.k_conveyorSlowInRPS : IndexerConstants.k_conveyorSlowOutRPS));    
        }, () -> {
                m_indexerConveyorTop.setControl(m_brakeRequest);
                m_indexerConveyorBottom.setControl(m_brakeRequest);
            }, this); 
    }

    public Command runPower(){
        return Commands.runEnd(() -> {
            
            m_indexerConveyorBottom.setThrottle(1.0);
            m_indexerConveyorTop.setThrottle(1.0);
        }, () -> {
            m_indexerConveyorTop.setParadoxControl(m_brakeRequest);
            m_indexerConveyorBottom.setParadoxControl(m_brakeRequest);
        }, this);
    }

    public double getVelocity() {
        return (m_indexerConveyorTop.getVelocity().getValueAsDouble() + m_indexerConveyorBottom.getVelocity().getValueAsDouble()) / 2;
    }

    @Override
    public void periodic() {
        Telemetry.log("Ball Tuhneull Velocity", m_indexerConveyorTop.getVelocity().getValueAsDouble());
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
