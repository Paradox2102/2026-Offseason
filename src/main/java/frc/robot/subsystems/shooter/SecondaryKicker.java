package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class SecondaryKicker extends SubsystemBase {
    private final CANBus k_CANBus = CANIDConstants.k_canBus3;

    private final TalonFX m_secondaryKickerMotor = new TalonFX(CANIDConstants.goUp, k_CANBus);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final double k_gearRatio = 1;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), ShooterConstants.k_upKickerMOI, k_gearRatio), DCMotor.getKrakenX60(1));

    public SecondaryKicker() {
        m_secondaryKickerMotor.getConfigurator().apply(ShooterConstants.k_secondaryKickerConfig);
        if (Utils.isSimulation()){
            var talonFXSim = m_secondaryKickerMotor.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }
    public Command run() {
        return Commands.runEnd(() -> {
                   
            m_secondaryKickerMotor.setControl(m_request.withVelocity(ShooterConstants.k_secondaryKickerRPS));
        }, () -> {m_secondaryKickerMotor.setThrottle(0);}, this); }
    
    public Command testRun(){
        return Commands.runEnd(() -> {
            m_secondaryKickerMotor.setThrottle(1.0);
        }, () -> m_secondaryKickerMotor.setControl(new StaticBrake()), this);
    }

    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_secondaryKickerMotor.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(m_secondaryKickerMotor.getMotorVoltage().getValue().unit()));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRotorAcceleration(m_motorSimModel.getAngularAcceleration() * k_gearRatio);
        talonFXSim.setRotorVelocity(Units.radiansToRotations(m_motorSimModel.getAngularVelocity()) * k_gearRatio);
            
        
        RobotContainer.m_telemetry.log("Upward Kicker Motor Speed", Units.radiansToRotations(m_motorSimModel.getAngularVelocity()));

    }
}