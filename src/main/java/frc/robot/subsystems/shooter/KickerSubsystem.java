package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;
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

public class KickerSubsystem extends SubsystemBase {
    private final CANBus k_CANBus = CANIDConstants.k_canBus3;

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final CoastOut m_coastRequest = new CoastOut();

    private final ParadoxTalon m_kickerMotor = new ParadoxTalon(CANIDConstants.kicker, k_CANBus, m_velocityRequest, 0.1);

    private final double k_gearRatio = 1;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), ShooterConstants.k_kickerMOI, k_gearRatio), DCMotor.getKrakenX60(1));

    public KickerSubsystem() {
        m_kickerMotor.getConfigurator().apply(ShooterConstants.k_kickerConfig);

        if (Utils.isSimulation()){
        var talonFXSim = m_kickerMotor.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }
    public Command run() {
        return Commands.runEnd(() -> {
            m_kickerMotor.setParadoxControl(m_velocityRequest.withVelocity(ShooterConstants.k_kickOutRPS));
        }, () -> {
            m_kickerMotor.setParadoxControl(m_coastRequest);
        }, this); }

        public Command runPower(){
        return Commands.runEnd(() -> {
            
            m_kickerMotor.setThrottle(0.9);
        }, () -> {
            m_kickerMotor.setParadoxControl(m_coastRequest);
        }, this);
    }

    @Override
    public void periodic() {
        Telemetry.log("Vertical Roller", m_kickerMotor.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = m_kickerMotor.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(m_kickerMotor.getMotorVoltage().getValue().unit()));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRotorAcceleration(m_motorSimModel.getAngularAcceleration() * k_gearRatio);
        talonFXSim.setRotorVelocity(Units.radiansToRotations(m_motorSimModel.getAngularVelocity()) * k_gearRatio);
            
        Telemetry.log("Kicker Motor Speed", Units.radiansToRotations(m_motorSimModel.getAngularVelocity()));
    }
}