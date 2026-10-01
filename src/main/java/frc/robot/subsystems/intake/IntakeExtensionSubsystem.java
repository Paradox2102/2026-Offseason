package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IntakeConstants;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;

public class IntakeExtensionSubsystem extends SubsystemBase {
    private static final double k_intakeMaxExtension = 20.0;//TODO: tune in sim
    private static final double k_elevatorDrimRadius = 0.038091; // If something is wrong then this is the first thing to check. Idk if I did this right
    private static final double k_elevatorMass = 5.401715051809;
    private final CANBus k_CANBus = CANIDConstants.k_canBus2;

    private final TalonFX m_intakeExtensionLeader = new TalonFX(CANIDConstants.intakeExtensionLeader, k_CANBus);
    private final TalonFX m_intakeExtensionFollower = new TalonFX(CANIDConstants.intakeExtensionFollower, k_CANBus); 

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    private final Follower m_follow = new Follower(CANIDConstants.intakeExtensionLeader, MotorAlignmentValue.Aligned);

    private final double k_gearRatio = 2.8;
    private final ElevatorSim m_motorSimModel = new ElevatorSim(
           Models.elevatorFromPhysicalConstants(
      DCMotor.getKrakenX60Foc(2), k_elevatorMass, k_elevatorDrimRadius, k_gearRatio),
       DCMotor.getKrakenX60Foc(2), 0.0, k_intakeMaxExtension, false, 0);


    public IntakeExtensionSubsystem() {
        m_intakeExtensionLeader.getConfigurator().apply(IntakeConstants.k_intakeExtensionConfigs);
        m_intakeExtensionFollower.setControl(m_follow);

        if (Utils.isSimulation()){
            var talonFXSim = m_intakeExtensionLeader.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    public Command retractIntake() {
        return Commands.run(() -> {
            m_intakeExtensionLeader.setControl(m_request.withPosition(0));
        }, this).until(() -> isIntakeRetracted());
    }
    public Command extendIntake() {
        return Commands.runOnce(() -> {
            m_intakeExtensionLeader.setControl(m_request.withPosition(IntakeConstants.k_outPosition));
            if (RobotBase.isSimulation()){
                
            }
        }, this);
    }

    public boolean isIntakeRetracted(){
        return m_intakeExtensionLeader.getPosition().getValueAsDouble() < 0;
    }

    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_intakeExtensionLeader.getSimState();

   // set the supply voltage of the TalonFX
   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   // get the motor voltage of the TalonFX
//    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
   m_motorSimModel.setInputVoltage(m_intakeExtensionLeader.getMotorVoltage().getValueAsDouble());
   m_motorSimModel.update(0.020); // assume 20 ms loop time

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   talonFXSim.setRawRotorPosition(m_motorSimModel.getPosition() * k_gearRatio);
    }
}
