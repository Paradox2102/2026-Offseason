package frc.robot.subsystems.intake;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.IntakeConstants;

public class IntakeExtensionSubsystem extends SubsystemBase {
    private final TalonFX m_intakeExtensionLeader = new TalonFX(CANIDConstants.intakeExtensionLeader);
    private final TalonFX m_intakeExtensionFollower = new TalonFX(CANIDConstants.intakeExtensionFollower);

    public Trigger m_intakeRetracted = new Trigger(() -> (m_intakeExtensionLeader.getPosition().getValueAsDouble() < 0));

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    private final double k_gearRatio = 2.8;
    private final DCMotorSim m_motorSimModel = new DCMotorSim(
   LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60Foc(2), 0.4214011009, k_gearRatio
   ),
   DCMotor.getKrakenX60Foc(2)
);

    public IntakeExtensionSubsystem() {
        m_intakeExtensionFollower.getConfigurator().apply(IntakeConstants.k_intakeRollerConfigs);
        m_intakeExtensionLeader.getConfigurator().apply(IntakeConstants.k_intakeRollerConfigs);

        if (Utils.isSimulation()){
            var talonFXSim = m_intakeExtensionLeader.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    public Command retractIntake() {
        return Commands.run(() -> {
            Globals.setFollowerPosition(m_intakeExtensionLeader, m_intakeExtensionFollower, m_request, 0, MotorAlignmentValue.Aligned);
        }, this).until(() -> m_intakeRetracted.getAsBoolean());
    }
    public Command extendIntake() {
        return Commands.runOnce(() -> {
            Globals.setFollowerPosition(m_intakeExtensionLeader, m_intakeExtensionFollower, m_request, IntakeConstants.k_outPosition, MotorAlignmentValue.Aligned);
        }, this);
    }
    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_intakeExtensionLeader.getSimState();

   // set the supply voltage of the TalonFX
   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   // get the motor voltage of the TalonFX
   var motorVoltage = talonFXSim.getMotorVoltageMeasure();

   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
   m_motorSimModel.setInputVoltage(m_intakeExtensionLeader.getMotorVoltage().getValueAsDouble());
   m_motorSimModel.update(0.020); // assume 20 ms loop time

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(k_gearRatio));
   talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(k_gearRatio));
    }
}
