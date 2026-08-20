package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.system.DCMotor;
// TODO - need to figure out how this is handled in 2027 - import org.wpilib.math.system.plant.LinearSystemId;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.utils.Constants.CANIDConstants;
import frc.robot.utils.Constants.Globals;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.Constants.IntakeConstants.IndexerConstants;

public class KickerSubsystem extends SubsystemBase {
    private final TalonFX m_kickerMotor = new TalonFX(CANIDConstants.kicker, new CANBus("1"));  // TODO - need to assign canbus numbers
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final double k_gearRatio = 1;

/* TODO - need to figure this out for 2027    
    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), ShooterConstants.k_kickerMOI, k_gearRatio), DCMotor.getKrakenX60(1));
*/

    public KickerSubsystem() {
        m_kickerMotor.getConfigurator().apply(new MotorOutputConfigs());

        if (Utils.isSimulation()){
        var talonFXSim = m_kickerMotor.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }
    public Command run(boolean in) {
        return Commands.run(() -> {               
            Globals.setRPS(m_kickerMotor, m_request, in ? ShooterConstants.k_kickInRPM : ShooterConstants.k_kickOutRPM);
        }, this); }
@Override
    public void simulationPeriodic() {
    var talonFXSim = m_kickerMotor.getSimState();

   // set the supply voltage of the TalonFX
   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   // get the motor voltage of the TalonFX
   var motorVoltage = talonFXSim.getMotorVoltageMeasure();

   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
/* TODO - need to figure this out for 2027   
   m_motorSimModel.setInputVoltage(m_kickerMotor.getMotorVoltage().getValueAsDouble());
   m_motorSimModel.update(0.020); // assume 20 ms loop time   

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(k_gearRatio));
*/   
    }
}