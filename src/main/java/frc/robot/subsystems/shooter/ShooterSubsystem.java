package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Sales_because_if_6328_does_if_it_must_be_good.SalesAssociate;
import frc.robot.utils.ParadoxTalon;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final Follower m_followRequest = new Follower(CANIDConstants.shooterLeader, MotorAlignmentValue.Opposed);
    private final CoastOut m_coastRequest = new CoastOut();

    private final CANBus k_CANBus = CANIDConstants.k_canBus3;
    private final ParadoxTalon m_shooterLeader = new ParadoxTalon(CANIDConstants.shooterLeader, k_CANBus, m_coastRequest, 0.1);
    private final ParadoxTalon m_shooterFollower = new ParadoxTalon(CANIDConstants.shooterFollower, k_CANBus, m_followRequest, 0.1);

    public final InterpolatingDoubleTreeMap m_shooterLerpTable = new InterpolatingDoubleTreeMap();

    private final double k_gearRatio = 1;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), ShooterConstants.k_shooterMOI, k_gearRatio), DCMotor.getKrakenX60(1));


    public boolean isAtSpeed() {
        return Math.abs((Math.abs(m_shooterLeader.getVelocity().getValueAsDouble()) + Math.abs(m_shooterFollower.getVelocity().getValueAsDouble())) / 2 - m_velocityRequest.Velocity) < 3;
    }
    public ShooterSubsystem() {
        m_shooterLeader.getConfigurator().apply(ShooterConstants.k_shooterConfigs);
        m_shooterLeader.configNeutralMode(NeutralModeValue.Coast);
        

        for (double[] shotPower : ShooterConstants.k_scoreShotSpeeds){
            m_shooterLerpTable.put(shotPower[0], shotPower[1]);
        
        if (Utils.isSimulation()){
            var talonFXSim = m_shooterLeader.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
            }
        }
    }

    public Command shoot(DoubleSupplier distanceToTarget) {
        return Commands.runEnd(() -> {               
            m_shooterLeader.setParadoxControl(m_velocityRequest.withVelocity(m_shooterLerpTable.get(distanceToTarget.getAsDouble())));
            SalesAssociate.sayByeToFuel();
        }, () -> {
            m_shooterLeader.setParadoxControl(m_coastRequest);
        }, this);
    }

    public Command staticShoot(){
        return Commands.runEnd(() -> {

            m_shooterLeader.setParadoxControl(m_velocityRequest.withVelocity(62));
        }, () -> {
            
            m_shooterLeader.setParadoxControl(m_coastRequest);
        }, this);
    }

    public Command shootFull(){
        return Commands.run(() -> m_shooterLeader.setThrottle(0.8), this);
    }

    @Override
    public void periodic() {
        Telemetry.log("Shooter Speed", (Math.abs(m_shooterLeader.getVelocity().getValueAsDouble()) + Math.abs(m_shooterFollower.getVelocity().getValueAsDouble())) / 2);
        Telemetry.log("Is Shooter At Speed", isAtSpeed());
    }
    @Override
    public void simulationPeriodic() {
        var talonFXSim = m_shooterLeader.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(m_shooterLeader.getMotorVoltage().getValue().unit()));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRotorAcceleration(m_motorSimModel.getAngularAcceleration() * k_gearRatio);
        talonFXSim.setRotorVelocity(Units.radiansToRotations(m_motorSimModel.getAngularVelocity()) * k_gearRatio);
            
        //Multipled by 0.159 to convert rad/s to rps
        Telemetry.log("Shooter Motor Speed", Units.radiansToRotations(m_motorSimModel.getAngularVelocity()));


    }
}
