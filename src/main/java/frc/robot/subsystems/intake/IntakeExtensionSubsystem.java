package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Sales_because_if_6328_does_if_it_must_be_good.SalesAssociate;
import frc.robot.utils.ParadoxTalon;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class IntakeExtensionSubsystem extends SubsystemBase {
    private static final double k_intakeMaxExtension = 3.95;
    private static final double k_elevatorDrimRadius = 0.038091; // If something is wrong then this is the first thing to check. Idk if I did this right
    private static final double k_elevatorMass = 5.401715051809;
    private final CANBus k_CANBus = CANIDConstants.k_canBus4;

    private final MotionMagicVelocityVoltage m_velocityRequest = new MotionMagicVelocityVoltage(0);
    private final Follower m_followRequest = new Follower(CANIDConstants.intakeExtensionLeader, MotorAlignmentValue.Opposed);
    private final StaticBrake m_brakeRequest = new StaticBrake();
    
    private final ParadoxTalon m_intakeExtensionLeader = new ParadoxTalon(CANIDConstants.intakeExtensionLeader, k_CANBus, m_brakeRequest, 0.1);
    private final ParadoxTalon m_intakeExtensionFollower = new ParadoxTalon(CANIDConstants.intakeExtensionFollower, k_CANBus, m_brakeRequest, 0.1); 

    

    private final double k_gearRatio = 2.8;
    private final ElevatorSim m_motorSimModel = new ElevatorSim(
           Models.elevatorFromPhysicalConstants(
      DCMotor.getKrakenX60Foc(2), k_elevatorMass, k_elevatorDrimRadius, k_gearRatio),
       DCMotor.getKrakenX60Foc(2), 0.0, k_intakeMaxExtension, false, 0);

    
    public static enum TargetPosition {
        IN(0),
        OUT(k_intakeMaxExtension);

        double position;
        TargetPosition(double target){
            position = target;
        }
    }

    public IntakeExtensionSubsystem() {
        m_intakeExtensionLeader.getConfigurator().apply(IntakeConstants.m_intakeExtensionConfigs);
        m_intakeExtensionFollower.getConfigurator().apply(IntakeConstants.m_intakeExtensionConfigs);
        m_intakeExtensionFollower.setParadoxControl(m_followRequest);

        // m_intakeExtensionLeader.setPosition(0);
        // m_intakeExtensionFollower.setPosition(0);

        if (Utils.isSimulation()){
            var talonFXSim = m_intakeExtensionLeader.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    @Override
    public void periodic() {
        Telemetry.log("Intake Extension at target", isIntakeAtTarget(TargetPosition.OUT));

        Telemetry.log("Intake Extension Position", (m_intakeExtensionLeader.getPosition().getValueAsDouble() + m_intakeExtensionFollower.getPosition().getValueAsDouble()) / 2);
    }

    // public Command moveIntake(TargetPosition position){
    //     return Commands.run(() -> {
    //         m_intakeExtensionLeader.setParadoxControl(m_positionRequest.withPosition(position.position));
    //     }, this).until(() -> isIntakeAtTarget(position)).finallyDo(() -> m_intakeExtensionLeader.setParadoxControl(m_brakeRequest));
    // }

    public Command crush(){
        return Commands.repeatingSequence(extendIntakePower(), retractIntakePower());
    }

    public Command retractIntake() {
        return Commands.run(() -> {
            m_intakeExtensionLeader.setParadoxControl(m_velocityRequest.withVelocity(-10));
            SalesAssociate.persuasion();
        }
        ).until(() -> isIntakeAtTarget(TargetPosition.IN)).finallyDo(() -> m_intakeExtensionLeader.setThrottle(0));
    }

    public Command extendIntake() {
        return Commands.run(() -> {
            m_intakeExtensionLeader.setParadoxControl(m_velocityRequest.withVelocity(10));
        }, this).until(() -> isIntakeAtTarget(TargetPosition.OUT)).finallyDo(() -> m_intakeExtensionLeader.setThrottle(0));
    }
    
    public Command retractIntakePower() {
        return Commands.runOnce(() -> {
            m_intakeExtensionLeader.setThrottle(-0.5);
            m_intakeExtensionFollower.setThrottle(0.5);
        }, this).until(() -> isIntakeAtTarget(TargetPosition.IN)).finallyDo(() -> m_intakeExtensionLeader.setThrottle(0));
    }

    public Command extendIntakePower() {
        return Commands.runOnce(() -> {
            m_intakeExtensionLeader.setThrottle(0.5);
        }, this).until(() -> isIntakeAtTarget(TargetPosition.OUT)).finallyDo(() -> m_intakeExtensionLeader.setThrottle(0));
    }

    public boolean isIntakeAtTarget(TargetPosition target){
        return Math.abs(((m_intakeExtensionLeader.getPosition().getValueAsDouble() + m_intakeExtensionFollower.getPosition().getValueAsDouble()) / 2) - target.position) < 0.08;

    }

    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_intakeExtensionLeader.getSimState();

   // set the supply voltage of the TalonFX
   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   // get the motor voltage of the TalonFX
   // var motorVoltage = talonFXSim.getMotorVoltageMeasure();

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
