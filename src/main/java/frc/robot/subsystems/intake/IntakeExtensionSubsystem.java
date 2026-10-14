package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.ParadoxTalon;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class IntakeExtensionSubsystem extends SubsystemBase {
    private static final double k_intakeMaxExtension = 4.06;
    private static final double k_elevatorDrimRadius = 0.038091; // If something is wrong then this is the first thing to check. Idk if I did this right
    private static final double k_elevatorMass = 5.401715051809;
    private final CANBus k_CANBus = CANIDConstants.k_canBus4;

    private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);
    private final Follower m_followRequest = new Follower(CANIDConstants.intakeExtensionLeader, MotorAlignmentValue.Opposed);
    private final StaticBrake m_brakeRequest = new StaticBrake();
    
    private final ParadoxTalon m_intakeExtensionLeader = new ParadoxTalon(CANIDConstants.intakeExtensionLeader, k_CANBus, m_brakeRequest, 0.1);
    private final ParadoxTalon m_intakeExtensionFollower = new ParadoxTalon(CANIDConstants.intakeExtensionFollower, k_CANBus, m_brakeRequest, 0.1); 

    

    private final double k_gearRatio = 2.8;
    private final ElevatorSim m_motorSimModel = new ElevatorSim(
           Models.elevatorFromPhysicalConstants(
      DCMotor.getKrakenX60Foc(2), k_elevatorMass, k_elevatorDrimRadius, k_gearRatio),
       DCMotor.getKrakenX60Foc(2), 0.0, k_intakeMaxExtension, false, 0);

    private final Timer m_stallTimer = new Timer();
    private final double k_stallTimerThreshold = 0.15;
    
    public static enum targetPosition {
        IN(0),
        OUT(4.06);

        double position;
        targetPosition(double target){
            position = target;
        }
    }

    public IntakeExtensionSubsystem() {
        m_intakeExtensionLeader.getConfigurator().apply(IntakeConstants.k_intakeExtensionConfigs);
        m_intakeExtensionFollower.getConfigurator().apply(IntakeConstants.k_intakeExtensionConfigs);
        m_intakeExtensionFollower.setControl(m_followRequest);
        m_stallTimer.start();

        m_intakeExtensionLeader.setPosition(0);

        if (Utils.isSimulation()){
            var talonFXSim = m_intakeExtensionLeader.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    @Override
    public void periodic() {
        
        Telemetry.log("Intake Extension Position", m_intakeExtensionLeader.getPosition(true).getValueAsDouble());
    }

    public Command moveIntake(targetPosition position){
        return Commands.run(() -> {
            m_intakeExtensionLeader.setControl(m_positionRequest.withPosition(position.position));
        }, this).until(() -> isIntakeAtTarget(position)).finallyDo(() -> m_intakeExtensionLeader.setControl(m_brakeRequest));
    }

    public Command crush(){
        return Commands.repeatingSequence(moveIntake(targetPosition.IN), moveIntake(targetPosition.OUT));
    }

    public Command forceRetractIntake(){
        return Commands.startEnd(() -> m_intakeExtensionLeader.setThrottle(-1), () ->{
            m_intakeExtensionLeader.setThrottle(0);
            m_intakeExtensionLeader.setPosition(0);
        });
    }
    
    public Command retractIntakePower() {
        return Commands.startRun(() -> {
            m_stallTimer.reset();
            m_intakeExtensionLeader.setThrottle(-0.5);
        }, () -> {
            if (Math.abs(m_intakeExtensionFollower.getVelocity().getValueAsDouble()) > 0.05)
                m_stallTimer.reset();
        }, this).until(() -> m_stallTimer.get() > k_stallTimerThreshold).finallyDo(() -> m_intakeExtensionLeader.setThrottle(0));
    }

    public Command extendIntakePower() {
        return Commands.startRun(() -> {
            m_stallTimer.reset();
            m_intakeExtensionLeader.setThrottle(0.5);
        }, () -> {
            if (Math.abs(m_intakeExtensionFollower.getVelocity().getValueAsDouble()) > 0.05)
                m_stallTimer.reset();
        }, this).until(() -> m_stallTimer.get() > k_stallTimerThreshold).finallyDo(() -> m_intakeExtensionLeader.setThrottle(0));
    }

    public boolean isIntakeAtTarget(targetPosition target){
        return (Math.abs(m_intakeExtensionLeader.getPosition().getValueAsDouble() - m_intakeExtensionFollower.getPosition().getValueAsDouble()) / 2) - target.position < 0.1;
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
