package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import javax.naming.ldap.SortResponseControl;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.system.DCMotor;
// TODO - need to figure out how this is handled in 2027 - import org.wpilib.math.system.plant.LinearSystemId;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private final TalonFX m_shooterLeader = new TalonFX(CANIDConstants.shooterLeader, new CANBus("1")); // TODO - need to assign canbus numbers
    private final TalonFX k_shooterFollower = new TalonFX(CANIDConstants.shooterFollower, new CANBus("1")); // TODO - should we be creating a second instance or use a copy of the one created above?
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final InterpolatingDoubleTreeMap m_shooterLerpTable = new InterpolatingDoubleTreeMap();

    private final double k_gearRatio = 1;

/* TODO - need to figure this out for 2027
    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), ShooterConstants.k_shooterMOI, k_gearRatio), DCMotor.getKrakenX60(1));
*/

    public ShooterSubsystem() {
        k_shooterFollower.getConfigurator().apply(ShooterConstants.k_shooterConfigs);
        m_shooterLeader.getConfigurator().apply(ShooterConstants.k_shooterConfigs);

        for (double[] shotPower : ShooterConstants.k_shotSpeeds){
            m_shooterLerpTable.put(shotPower[0], shotPower[1]);
        
        if (Utils.isSimulation()){
        var talonFXSim = m_shooterLeader.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        }
    }

    public Command shoot(DoubleSupplier distanceToHub) {
    return Commands.run(() -> {               
        Globals.setFollowerRPM(m_shooterLeader, k_shooterFollower, m_request, m_shooterLerpTable.get(distanceToHub.getAsDouble()),
        MotorAlignmentValue.Aligned);   
    }, this); }



    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_shooterLeader.getSimState();

   // set the supply voltage of the TalonFX
   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   // get the motor voltage of the TalonFX
   var motorVoltage = talonFXSim.getMotorVoltageMeasure();

/* TODO - need to figure this out for 2027
   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
   m_motorSimModel.setInputVoltage(m_shooterLeader.getMotorVoltage().getValueAsDouble());
   m_motorSimModel.update(0.020); // assume 20 ms loop time

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(k_gearRatio));
*/   
    }
}
