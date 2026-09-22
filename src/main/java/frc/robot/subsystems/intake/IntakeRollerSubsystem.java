package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.RobotController;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final CANBus k_CANBus = CANBus.systemcore(2);

    private final TalonFX m_intakeRollerMotor = new TalonFX(CANIDConstants.intakeRoller, k_CANBus);
    
    private final BangBangController k_bangBang = new BangBangController();
    private final SimpleMotorFeedforward k_feedForward = new SimpleMotorFeedforward(IntakeConstants.k_rollerS, IntakeConstants.k_rollerV);

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
        return Commands.runEnd(() -> {
           m_intakeRollerMotor.configNeutralMode(NeutralModeValue.Coast);
            // m_intakeRollerMotor.setControl(new VelocityVoltage((k_bangBang.calculate(m_intakeRollerMotor.getVelocity(true).getValueAsDouble(), in ? IntakeConstants.k_inRPS : IntakeConstants.k_outRPS)* k_feedForward.calculate(in ? IntakeConstants.k_inRPS : IntakeConstants.k_outRPS))));
            // m_intakeRollerMotor.setControl(new VelocityVoltage(in ? IntakeConstants.k_inRPS : IntakeConstants.k_outRPS));
            // Globals.bangBang(IntakeConstants.k_inRPS, IntakeConstants.k_rollerV, m_intakeRollerMotor);
            // m_intakeRollerMotor.setControl(new VelocityVoltage(k_feedForward.calculate(IntakeConstants.k_inRPS)));
            m_intakeRollerMotor.setThrottle(k_bangBang.calculate(k_bangBang.calculate(m_intakeRollerMotor.getVelocity(true).getValueAsDouble(), in ? IntakeConstants.k_inRPS : IntakeConstants.k_outRPS)));
        }, () -> {
            m_intakeRollerMotor.configNeutralMode(NeutralModeValue.Brake);
            m_intakeRollerMotor.setControl(new StaticBrake());
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
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(m_intakeRollerMotor.getMotorVoltage().getValue().unit()));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRotorAcceleration(m_motorSimModel.getAngularAcceleration() * k_gearRatio);
        talonFXSim.setRotorVelocity((m_motorSimModel.getAngularVelocity() * 0.1591549430919) * k_gearRatio);
            
        //Multipled by 0.159 to convert rad/s to rps
        SmartDashboard.putNumber("Intake Roller Motor Speed", m_motorSimModel.getAngularVelocity() * 0.1591549430919);

    }
}
