package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.ShooterConstants;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
    private final CANBus k_CANBus = CANBus.systemcore(1);

    private final TalonFX m_kickerMotor = new TalonFX(CANIDConstants.kicker, k_CANBus);  // TODO - need to assign canbus numbers
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final double k_gearRatio = 1;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), ShooterConstants.k_kickerMOI, k_gearRatio), DCMotor.getKrakenX60(1));

    public KickerSubsystem() {
        m_kickerMotor.getConfigurator().apply(new MotorOutputConfigs());

        if (Utils.isSimulation()){
        var talonFXSim = m_kickerMotor.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }
    public Command run(boolean in) {
        return Commands.run(() -> {               
            Globals.setRPS(m_kickerMotor, m_request, in ? ShooterConstants.k_kickInRPS : ShooterConstants.k_kickOutRPS);
        }, this); }
@Override
    public void simulationPeriodic() {
    var talonFXSim = m_kickerMotor.getSimState();

   // set the supply voltage of the TalonFX
   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   // get the motor voltage of the TalonFX
//    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
   m_motorSimModel.setInputVoltage(m_kickerMotor.getMotorVoltage().getValueAsDouble());
   m_motorSimModel.update(0.020); // assume 20 ms loop time   

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity() * k_gearRatio);
    SmartDashboard.putNumber("Kicker Motor Speed", m_kickerMotor.getVelocity().getValueAsDouble());

    }
}