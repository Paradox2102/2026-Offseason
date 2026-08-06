package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.ShooterConstants;

public class HoodSubsystem extends SubsystemBase{

    private final TalonFX m_hoodMotor = new TalonFX(CANIDConstants.hood);
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
        for (double[] shotAngle : ShooterConstants.m_hoodAngle){
            m_hoodLerpTable.put(shotAngle[0], shotAngle[1]);
        }
        m_hoodMotor.getConfigurator().apply(ShooterConstants.k_hoodConfig);

        if (Utils.isSimulation()){
            var talonFXSim = m_hoodMotor.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    public Command pitchHood(double distanceToHub) {
        return Commands.run(() -> {
            Globals.setPosition(m_hoodMotor, m_request, m_hoodLerpTable.get(distanceToHub));
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
   talonFXSim.setRawRotorPosition(m_motorSimModel.getAngleRads() * k_gearRatio);
   talonFXSim.setRotorVelocity(m_motorSimModel.getVelocityRadPerSec() * k_gearRatio);
    }
}
