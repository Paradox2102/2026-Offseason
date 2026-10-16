package frc.robot.subsystems.shooter;


import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.units.measure.Angle;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utils.DoubleNumber;
import frc.robot.utils.ParadoxTalon;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;


public class TurretAimSubsystem extends SubsystemBase{
    private final CANBus k_CANBus = CANIDConstants.k_canBus1;

    public final PositionVoltage m_positionRequest = new PositionVoltage(0);
    public final CoastOut m_coastRequest = new CoastOut();

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(
            Constants.ShooterConstants.k_turretConfig.Slot0.kP,
            Constants.ShooterConstants.k_turretConfig.Slot0.kI,
            Constants.ShooterConstants.k_turretConfig.Slot0.kD
        )
        .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(
            Constants.ShooterConstants.k_turretConfig.Slot0.kP,
            Constants.ShooterConstants.k_turretConfig.Slot0.kI,
            Constants.ShooterConstants.k_turretConfig.Slot0.kD
        )
        // Telemetry name and verbosity level
        .withTelemetry("Turret Angle", TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
        .withGearing(new MechanismGearing(GearBox.fromTeeth(95, 14)))
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        // Starting position is where your arm starts
        .withStartingPosition(Degrees.of(-5))
        // Soft limit is applied to the SmartMotorControllers PID
        .withSoftLimits(Degrees.of(-20), Degrees.of(10));


    private final ParadoxTalon m_shooterAzimuth = new ParadoxTalon(CANIDConstants.shooterAzimuth, k_CANBus, m_coastRequest, 0.1);
    
    private SmartMotorController azimuthSmartMotorController = new TalonFXWrapper(m_shooterAzimuth, DCMotor.getNEO(1), smcConfig);
    
    private ArmConfig armCfg = new ArmConfig()
        // Hard limit is applied to the simulation.
        .withHardLimits(Degrees.of(-30), Degrees.of(40))
        // Length and mass of your arm for sim.
        .withLength(Feet.of(1))
        // Telemetry name and verbosity for the arm.
        .withTelemetry("Turret Aim", TelemetryVerbosity.HIGH);

    // Arm Mechanism
    private Arm m_azimuthArm = new Arm(armCfg, azimuthSmartMotorController);


    public boolean m_isOnTarget() {
        return Math.abs(m_azimuthArm.getAngle().baseUnitMagnitude() - getTargetRotation().baseUnitMagnitude()) < 0.1;
    }

 private final SingleJointedArmSim m_motorSimModel = new SingleJointedArmSim(Models.singleJointedArmFromPhysicalConstants(DCMotor.getKrakenX60(1), 0.04969116, ShooterConstants.k_shooterGearReduction), DCMotor.getKrakenX60(1), ShooterConstants.k_shooterGearReduction, 0.1, Units.degreesToRadians(ShooterConstants.k_turretBounds.getMinBound()), Units.degreesToRadians(ShooterConstants.k_turretBounds.getMaxBound()), false, 0);   
 public TurretAimSubsystem() {
        //m_shooterAzimuth.getConfigurator().apply(ShooterConstants.k_turretConfig);
    }

    public Command rotateToTarget() {
        return 
            m_azimuthArm.runTo(getTargetRotation(), Degrees.of(0.1));
            // m_shooterAzimuth.setRequest(m_positionRequest.withPosition(getTargetRotation()));
            // m_shooterAzimuth.applyRequest();
        
    }

    public Command staticAim() {
        return Commands.run(() -> {
            m_shooterAzimuth.setControl(m_positionRequest.withPosition(0.1 * ShooterConstants.k_shooterGearReduction)); 
        }, this);
    }

    private Angle getTargetRotation(){ 
        Pose3d hubPose = new Pose3d(Subsystems.m_swerveSubsystem.getSotmHubAimPos()).plus(new Transform3d(ShooterConstants.k_turretOffset, new Rotation3d(0, 0, Subsystems.m_swerveSubsystem.getSotmHubAimPos().getRotation().getRadians())));

        var rotationTriangle = new Translation2d(hubPose.getX() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getX() : FieldConstants.k_blueHub.getX()), (hubPose.getY() - (SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub.getY() : FieldConstants.k_blueHub.getY())));
        
        var targetRotation = Math.atan2(rotationTriangle.getY(), rotationTriangle.getX());
        double bestRotation = targetRotation;
        for (int i = -1; i <= 1; i++) {
            bestRotation = (checkBestRotation(bestRotation, targetRotation + (i * (2 * Math.PI)), hubPose.toPose2d()) * ShooterConstants.k_shooterGearReduction);
        }
        return Degrees.of(Units.radiansToDegrees(bestRotation));
    }

    private double checkBestRotation(double bestRotation, double testRotation, Pose2d hubPose){
        var robotRotation = hubPose.getRotation().getRadians();
        if (DoubleNumber.isBetween(testRotation, ShooterConstants.k_turretBounds)){
            if ((testRotation - robotRotation) < (bestRotation - robotRotation)){
                bestRotation = testRotation;
            }
        }
        
        return bestRotation;

    }

    public TalonFX getShooterAzimuth(){
        return m_shooterAzimuth;

    }
    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_shooterAzimuth.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(m_shooterAzimuth.getMotorVoltage().getValue().unit()));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRotorVelocity(m_motorSimModel.getVelocity() * 0.15915495);
        talonFXSim.setRawRotorPosition(Units.radiansToRotations(m_motorSimModel.getAngle()));
            
        Telemetry.log("Turret Angle Rotations", Units.radiansToRotations(m_motorSimModel.getAngle() * ShooterConstants.k_shooterGearReduction));

    }


}
