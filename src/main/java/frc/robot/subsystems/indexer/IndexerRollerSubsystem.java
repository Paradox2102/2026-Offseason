package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IndexerConstants;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class IndexerRollerSubsystem extends SubsystemBase {
    private final CANBus k_CANBus = CANBus.systemcore(3);
    private final TalonFX m_indexerRollerModer = new TalonFX(CANIDConstants.indexerRoller, k_CANBus);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final double k_gearRatio = 0.31;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), IndexerConstants.k_indexerRollerMOI, k_gearRatio), DCMotor.getKrakenX60(1));


    public IndexerRollerSubsystem() {
        m_indexerRollerModer.getConfigurator().apply(new MotorOutputConfigs());

        if (Utils.isSimulation()){
            var talonFXSim = m_indexerRollerModer.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }
    public Command run(boolean in) {
        return Commands.run(() -> {               
            m_indexerRollerModer.setControl(m_request.withFeedForward(in ? IndexerConstants.k_rollerInRPS : IndexerConstants.k_rollerOutRPS));
        }, this); }
    
    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_indexerRollerModer.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(m_indexerRollerModer.getMotorVoltage().getValue().unit()));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRotorAcceleration(m_motorSimModel.getAngularAcceleration() * k_gearRatio);
        talonFXSim.setRotorVelocity((m_motorSimModel.getAngularVelocity() * 0.1591549430919) * k_gearRatio);
            
        //Multipled by 0.159 to convert rad/s to rps
        SmartDashboard.putNumber("Indexer Roller Motor Speed", m_motorSimModel.getAngularVelocity() * 0.1591549430919);

    }
}
