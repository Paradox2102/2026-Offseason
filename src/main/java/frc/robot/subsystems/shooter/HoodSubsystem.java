package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.system.DCMotor;
// TODO - need to figure out how this is handled in 2027 - import org.wpilib.math.system.plant.LinearSystemId;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.ShooterConstants;

public class HoodSubsystem extends SubsystemBase{

    private final TalonFX m_hoodMotor = new TalonFX(CANIDConstants.hood, new CANBus("1"));  // TODO - need to assign canbus numbers
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    private final InterpolatingDoubleTreeMap m_hoodLerpTable = new InterpolatingDoubleTreeMap();

    private final DigitalInput m_limitSwitch = new DigitalInput(0);

    private final double k_gearRatio = 5;
    private final SingleJointedArmSim m_motorSimModel = new SingleJointedArmSim(DCMotor.getKrakenX44(1),
    k_gearRatio, ShooterConstants.k_hoodMOI, ShooterConstants.k_hoodArmLength, 0, ShooterConstants.k_hoodMaxAngle, true, 0);

    //    LinearSystemId.createDCMotorSystem(
//       DCMotor.getKrakenX44Foc(1), 0.0254713554, k_gearRatio
//    ),
//    DCMotor.getKrakenX44Foc(1)
// );

    @Override
    public void periodic() {
        if (m_limitSwitch.get()){
            m_hoodMotor.setPosition(0);
        }
    }
    public HoodSubsystem() {
        for (double[] shotAngle : ShooterConstants.k_hoodAngle){
            m_hoodLerpTable.put(shotAngle[0], shotAngle[1]);
        }
        m_hoodMotor.getConfigurator().apply(ShooterConstants.k_hoodConfig);

        if (Utils.isSimulation()){
            var talonFXSim = m_hoodMotor.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    public Command pitchHood(DoubleSupplier distanceToHub) {
        return Commands.run(() -> {
            Globals.setPosition(m_hoodMotor, m_request, m_hoodLerpTable.get(distanceToHub.getAsDouble()));
        }, this);
    }

    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_hoodMotor.getSimState();

   // set the supply voltage of the TalonFX
   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   // get the motor voltage of the TalonFX
   var motorVoltage = talonFXSim.getMotorVoltageMeasure();

   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
   m_motorSimModel.setInputVoltage(m_hoodMotor.getMotorVoltage().getValueAsDouble());
   m_motorSimModel.update(0.020); // assume 20 ms loop time

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
/* TODO - need to figure out how this has changed for 2027   
   talonFXSim.setRawRotorPosition(m_motorSimModel.getAngleRads() * k_gearRatio);
   talonFXSim.setRotorVelocity(m_motorSimModel.getVelocityRadPerSec() * k_gearRatio);
*/   
    }
}
