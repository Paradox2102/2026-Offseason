package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.IntakeConstants;

import org.wpilib.math.system.Models;
import org.wpilib.math.system.DCMotor;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final CANBus k_CANBus = CANBus.systemcore(2);

    private final TalonFX m_intakeRollerMotor = new TalonFX(CANIDConstants.intakeRoller, k_CANBus); // TODO - Need to assign canbus numbers
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    
    private final double k_gearRatio = 0.5;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), IntakeConstants.k_intakeRollerMOI, k_gearRatio), DCMotor.getKrakenX60(1));
    public IntakeRollerSubsystem() {
        m_intakeRollerMotor.getConfigurator().apply(IntakeConstants.k_intakeRollerConfigs);
        
        if (Utils.isSimulation()){
            var talonFXSim = m_intakeRollerMotor.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    public Command run(boolean in){
        return Commands.run(() -> {
            Globals.setRPS(m_intakeRollerMotor, m_request, in ? IntakeConstants.k_inRPM : IntakeConstants.k_outRPM);
        }, this);
    }

    public Command stop(){
        return Commands.runOnce(() -> m_intakeRollerMotor.setVoltage(0), this);
    }
    
    
    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_intakeRollerMotor.getSimState();

   // set the supply voltage of the TalonFX
   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   // get the motor voltage of the TalonFX
//    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
   m_motorSimModel.setInputVoltage(m_intakeRollerMotor.getMotorVoltage().getValueAsDouble());
   m_motorSimModel.update(0.020); // assume 20 ms loop time

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity() * k_gearRatio);
    SmartDashboard.putNumber("Intake Roller Moder Speed", m_intakeRollerMotor.getVelocity().getValueAsDouble());

    }
}
