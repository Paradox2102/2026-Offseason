package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.RobotContainer;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IntakeConstants;

import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.math.controller.BangBangController;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.units.measure.Velocity;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
    private static final StaticBrake k_brakeRequest = new StaticBrake();

    private final CANBus k_CANBus = CANIDConstants.k_canBus4;

    private final TalonFX m_intakeRollerMotor = new TalonFX(CANIDConstants.intakeRoller, k_CANBus);
    
    private final BangBangController k_bangBang = new BangBangController();

    private final double k_gearRatio = 0.5;

    private final VelocityVoltage m_request = new VelocityVoltage(0);

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
            m_intakeRollerMotor.setThrottle(k_bangBang.calculate(m_intakeRollerMotor.getVelocity(true).getValueAsDouble(), in ? IntakeConstants.k_inRPS : IntakeConstants.k_outRPS));
        }, () -> {
            m_intakeRollerMotor.configNeutralMode(NeutralModeValue.Brake);
            m_intakeRollerMotor.setControl(k_brakeRequest);
        }, this);
    }
    public Command testRun(boolean in){
        return Commands.runEnd(() -> m_intakeRollerMotor.setThrottle(0.7 * (in ? 1 : -1)), () -> m_intakeRollerMotor.setControl(k_brakeRequest), this);
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
        talonFXSim.setRotorVelocity(Units.radiansToRotations(m_motorSimModel.getAngularVelocity()) * k_gearRatio);
            
        Telemetry.log("Intake Roller Motor Speed", Units.radiansToRotations(m_motorSimModel.getAngularVelocity()));

    }
}
