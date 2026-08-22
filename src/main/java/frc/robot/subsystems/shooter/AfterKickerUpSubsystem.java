package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class AfterKickerUpSubsystem extends SubsystemBase {
    private final CANBus k_CANBus = CANBus.systemcore(1);

    private final TalonFX m_goUpMotor = new TalonFX(CANIDConstants.goUp, k_CANBus);  // TODO - need to assign the canbus numbers
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final double k_gearRatio = 1;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), ShooterConstants.k_upKickerMOI, k_gearRatio), DCMotor.getKrakenX60(1));

    public AfterKickerUpSubsystem() {
        m_goUpMotor.getConfigurator().apply(new MotorOutputConfigs());

        if (Utils.isSimulation()){
            var talonFXSim = m_goUpMotor.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }
    public Command run() {
        return Commands.run(() -> {               
            m_goUpMotor.setControl(m_request.withFeedForward(ShooterConstants.k_preShootRPM));
        }, this); }
    
    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_goUpMotor.getSimState();

   // set the supply voltage of the TalonFX
   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   // get the motor voltage of the TalonFX
//    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
   m_motorSimModel.setInputVoltage(m_goUpMotor.getMotorVoltage().getValueAsDouble());
   m_motorSimModel.update(0.020); // assume 20 ms loop time

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity() * k_gearRatio);
    }
}