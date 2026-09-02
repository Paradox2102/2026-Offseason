package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.IntakeConstants;

import org.wpilib.math.system.Models;
import org.wpilib.math.controller.BangBangController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final CANBus k_CANBus = CANBus.systemcore(2);

    private final TalonFX m_intakeRollerMotor = new TalonFX(CANIDConstants.intakeRoller, k_CANBus);
    private final BangBangController m_bangBang = new BangBangController();
    private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(IntakeConstants.k_rollerS, IntakeConstants.k_rollerV);
    private final double k_gearRatio = 0.5;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), IntakeConstants.k_intakeRollerMOI, k_gearRatio), DCMotor.getKrakenX60(1));
    public IntakeRollerSubsystem() {
        
        if (Utils.isSimulation()){
            var talonFXSim = m_intakeRollerMotor.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    public Command run(boolean in){
        return Commands.runEnd(() -> {
            // Globals.bangBang(in ? IntakeConstants.k_inRPM : IntakeConstants.k_outRPM, IntakeConstants.k_rollerV, m_intakeRollerMotor);
           m_intakeRollerMotor.configNeutralMode(NeutralModeValue.Coast);
            // m_intakeRollerMotor.setVoltage(m_bangBang.calculate(m_intakeRollerMotor.getVelocity().getValueAsDouble(), in ? IntakeConstants.k_inRPM : IntakeConstants.k_outRPM) * m_feedForward.calculate(in ? IntakeConstants.k_inRPM : IntakeConstants.k_outRPM));
            m_intakeRollerMotor.setVoltage(m_feedForward.calculate(in ? IntakeConstants.k_inRPS : IntakeConstants.k_outRPS));
            System.out.println("Test" + String.valueOf(m_intakeRollerMotor.getMotorVoltage().getValueAsDouble()));
        }, () -> {
            // Globals.bangBang(0, 0, m_intakeRollerMotor);
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
