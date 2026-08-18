package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import javax.naming.ldap.SortResponseControl;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.Constants.CANIDConstants;
import frc.robot.utils.Constants.Globals;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.Constants.Subsystems;
import frc.robot.utils.Constants.IntakeConstants.IndexerConstants;

public class ShooterSubsystem extends SubsystemBase{
    private final TalonFX m_shooterLeader = new TalonFX(CANIDConstants.shooterLeader);
    private final TalonFX m_shooterFollower = new TalonFX(CANIDConstants.shooterFollower);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final InterpolatingDoubleTreeMap m_shooterLerpTable = new InterpolatingDoubleTreeMap();

    private final double k_gearRatio = 1;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), ShooterConstants.k_shooterMOI, k_gearRatio), DCMotor.getKrakenX60(1));

    public final Trigger m_isAtSpeed = new Trigger(() -> (Math.abs((m_request.Velocity + m_shooterFollower.get()) / 2) - m_shooterLerpTable.get(Subsystems.m_swerveSubsystem.distanceToTarget())) < 100);
    public ShooterSubsystem() {
        m_shooterFollower.getConfigurator().apply(ShooterConstants.k_shooterConfigs);
        m_shooterLeader.getConfigurator().apply(ShooterConstants.k_shooterConfigs);

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
    return Commands.run(() -> {               
        Globals.setFollowerRPS(m_shooterLeader, m_shooterFollower, m_request, m_shooterLerpTable.get(distanceToTarget.getAsDouble()),
        MotorAlignmentValue.Aligned);   
    }, this); }

    public Command staticShoot(){
        return Commands.run(() -> Globals.setFollowerRPS(m_shooterLeader, m_shooterFollower, m_request, ShooterConstants.k_staticShotSpeed, MotorAlignmentValue.Aligned), this);
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
   m_motorSimModel.setInputVoltage(m_shooterLeader.getMotorVoltage().getValueAsDouble());
   m_motorSimModel.update(0.020); // assume 20 ms loop time

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(k_gearRatio));
    }
}
