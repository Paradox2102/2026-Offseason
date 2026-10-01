package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Subsystems;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private final CANBus k_CANBus = CANBus.systemcore(1);

    private final TalonFX m_shooterLeader = new TalonFX(CANIDConstants.shooterLeader, k_CANBus);
    private final TalonFX m_shooterFollower = new TalonFX(CANIDConstants.shooterFollower, k_CANBus);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    private final Follower m_follow = new Follower(CANIDConstants.shooterFollower, MotorAlignmentValue.Opposed);

    private final InterpolatingDoubleTreeMap m_shooterLerpTable = new InterpolatingDoubleTreeMap();

    private final double k_gearRatio = 1;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), ShooterConstants.k_shooterMOI, k_gearRatio), DCMotor.getKrakenX60(1));


    public boolean m_isAtSpeed() {
        return Math.abs((m_request.Velocity + m_shooterFollower.getVelocity().getValueAsDouble()) / 2) - m_shooterLerpTable.get(Subsystems.m_swerveSubsystem.distanceToTarget()) < 100;
    }
    public ShooterSubsystem() {
        m_shooterLeader.getConfigurator().apply(ShooterConstants.k_shooterConfigs);
        m_shooterFollower.setControl(m_follow);

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
            m_shooterLeader.setControl(m_request.withVelocity(m_shooterLerpTable.get(distanceToTarget.getAsDouble())));
        }, () -> {
            m_shooterLeader.setThrottle(0);
        }, this);
    }

    public Command staticShoot(){
        return Commands.runEnd(() -> { // TODO: Remove Magic Number
            m_shooterLeader.setControl(m_request.withVelocity(20));
        }, () -> {
            m_shooterLeader.setThrottle(0);
        }, this);
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
        SmartDashboard.putNumber("Shooter Motor Speed", Units.radiansToRotations(m_motorSimModel.getAngularVelocity()));


    }
}
