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
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.RepeatCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.WaitCommand;

public class IntakeExtensionSubsystem extends SubsystemBase {
    private static final double k_intakeMaxExtension = 3.95;
    private static final double k_intakeCrushPosition = 3.2;
    private static final double k_elevatorDrimRadius = 0.038091; // If something is wrong then this is the first thing to check. Idk if I did this right
    private static final double k_elevatorMass = 5.401715051809;
    private final CANBus k_CANBus = CANIDConstants.k_canBus4;

    private final MotionMagicVelocityVoltage m_velocityRequest = new MotionMagicVelocityVoltage(0);
    private final Follower m_followRequest = new Follower(CANIDConstants.intakeExtensionLeader, MotorAlignmentValue.Opposed);
    private final StaticBrake m_brakeRequest = new StaticBrake();
    
    private final ParadoxTalon m_intakeExtensionLeader = new ParadoxTalon(CANIDConstants.intakeExtensionLeader, k_CANBus, m_brakeRequest, 0.1, () -> !getExpectExtended());
    private final ParadoxTalon m_intakeExtensionFollower = new ParadoxTalon(CANIDConstants.intakeExtensionFollower, k_CANBus, m_brakeRequest, 0.1, () -> !getExpectExtended()); 

    private final Timer m_crushTimer = new Timer();
    private final double k_maxCrushTime = 1.0;

    private final double k_gearRatio = 2.8;

    private boolean m_expectExtended = false;

    private final ElevatorSim m_motorSimModel = new ElevatorSim(
           Models.elevatorFromPhysicalConstants(
      DCMotor.getKrakenX60Foc(2), k_elevatorMass, k_elevatorDrimRadius, k_gearRatio),
       DCMotor.getKrakenX60Foc(2), 0.0, k_intakeMaxExtension, false, 0);

    
    public static enum TargetPosition {
        IN(0.2),
        OUT(k_intakeMaxExtension),
        CRUSH(k_intakeCrushPosition);
        double position;
        TargetPosition(double target){
            position = target;
        }
    }

    public RepeatCommand agitate(){
        return new SequentialCommandGroup(retractIntakeCrush(), new WaitCommand(0.15), extendIntakeCrush()).repeatedly();
    }

    public IntakeExtensionSubsystem() {
        m_crushTimer.start();
        m_intakeExtensionLeader.getConfigurator().apply(IntakeConstants.m_intakeExtensionConfigs);
        m_intakeExtensionFollower.getConfigurator().apply(IntakeConstants.m_intakeExtensionConfigs);
        m_intakeExtensionFollower.setParadoxControl(m_followRequest);

        // m_intakeExtensionLeader.setPosition(k_intakeMaxExtension);
        // m_intakeExtensionFollower.setPosition(k_intakeMaxExtension);

        if (Utils.isSimulation()){
            var talonFXSim = m_intakeExtensionLeader.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    @Override
    public void periodic() {
        Telemetry.log("Intake Extension at target", isIntakeAtTarget(TargetPosition.OUT, false));

        Telemetry.log("Intake Extension Position", (m_intakeExtensionLeader.getPosition().getValueAsDouble() + m_intakeExtensionFollower.getPosition().getValueAsDouble()) / 2);
        Telemetry.log("BothIntakesMoving", Math.abs(m_intakeExtensionLeader.getVelocity().getValueAsDouble()) > 0.1 && Math.abs(m_intakeExtensionFollower.getVelocity().getValueAsDouble()) > 0.1);
    }

    // public Command moveIntake(TargetPosition position){
    //     return Commands.run(() -> {
    //         m_intakeExtensionLeader.setParadoxControl(m_positionRequest.withPosition(position.position));
    //     }, this).until(() -> isIntakeAtTarget(position)).finallyDo(() -> m_intakeExtensionLeader.setParadoxControl(m_brakeRequest));
    // }



    public Command retractIntake() {
        return Commands.run(() -> {
            m_intakeExtensionLeader.setParadoxControl(m_velocityRequest.withVelocity(-10));
            SalesAssociate.persuasion();
        }, this
        ).until(() -> isIntakeAtTarget(TargetPosition.IN, false)).finallyDo(() -> {m_intakeExtensionLeader.setControl(m_brakeRequest); m_expectExtended = false;});
    }

    public Command extendIntakeCrush() {
        return Commands.startRun(() -> {
            m_crushTimer.reset();
        }, () -> {
            m_intakeExtensionLeader.setParadoxControl(m_velocityRequest.withVelocity(6));
            
        }, this).until(() -> (isIntakeAtTarget(TargetPosition.OUT, false) || m_crushTimer.get() > k_maxCrushTime)).finallyDo(() -> {m_intakeExtensionLeader.setControl(m_brakeRequest); m_expectExtended = true;});
    }

        public Command retractIntakeCrush() {
        return Commands.startRun(() -> {
                m_crushTimer.reset();
        }, () -> {
            m_intakeExtensionLeader.setParadoxControl(m_velocityRequest.withVelocity(-6));
            SalesAssociate.persuasion();
        }
        ).until(() -> isIntakeAtTarget(TargetPosition.CRUSH, true) || m_crushTimer.get() > k_maxCrushTime).finallyDo(() -> m_intakeExtensionLeader.setControl(m_brakeRequest));
    }

    public Command extendIntake() {
        return Commands.run(() -> {
            m_intakeExtensionLeader.setParadoxControl(m_velocityRequest.withVelocity(10));
        }, this).until(() -> isIntakeAtTarget(TargetPosition.OUT, false)).finallyDo(() -> {m_intakeExtensionLeader.setControl(m_brakeRequest); m_expectExtended = true;});
    }

    public Command extendIntakeAndReset() {
        return Commands.sequence(extendIntake()).andThen(() -> {
            m_intakeExtensionLeader.setPosition(k_intakeMaxExtension);
            m_intakeExtensionFollower.setPosition(k_intakeMaxExtension);
        });
    }
    

    public boolean isIntakeAtTarget(TargetPosition target, boolean lessThan){
        var intakePosition = Math.abs((m_intakeExtensionLeader.getPosition().getValueAsDouble() + m_intakeExtensionFollower.getPosition().getValueAsDouble()) / 2);
        if (lessThan){
            return intakePosition < target.position;
        }
        Telemetry.log("Intake psotion - target", (intakePosition - target.position));
        return Math.abs(intakePosition - target.position) < 0.1;

    }

    public boolean getExpectExtended() {
        return m_expectExtended;
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
