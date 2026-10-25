package frc.robot.subsystems.shooter;


import org.photonvision.PhotonUtils;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utils.DoubleNumber;
import frc.robot.utils.ParadoxTalon;

public class TurretAimSubsystem extends SubsystemBase{
    private final CANBus k_CANBus = CANIDConstants.k_canBus3;

    public final PositionVoltage m_positionRequest = new PositionVoltage(0).withEnableFOC(true);
    public final CoastOut m_coastRequest = new CoastOut();

    private final ParadoxTalon m_shooterAzimuth = new ParadoxTalon(CANIDConstants.shooterAzimuth, k_CANBus, m_coastRequest, 0.1);
    

    public boolean isOnTarget() {
        return Math.abs(m_shooterAzimuth.getPosition().getValueAsDouble() - getTargetRotation()) < 0.5;
    }

 private final SingleJointedArmSim m_motorSimModel = new SingleJointedArmSim(Models.singleJointedArmFromPhysicalConstants(DCMotor.getKrakenX60(1), 0.04969116, ShooterConstants.k_shooterGearReduction), DCMotor.getKrakenX60(1), ShooterConstants.k_shooterGearReduction, 0.1, Units.degreesToRadians(ShooterConstants.k_turretBounds.getMinBound()), Units.degreesToRadians(ShooterConstants.k_turretBounds.getMaxBound()), false, 0);   
 public TurretAimSubsystem() {
        m_shooterAzimuth.getConfigurator().apply(ShooterConstants.k_turretConfig);
    }

    public Command rotateToTarget() {
        return Commands.run(() -> {
            m_shooterAzimuth.setParadoxControl(m_positionRequest.withPosition(getTargetRotation()));
        }, this);
    }

    public Command staticAim() {
        return Commands.run(() -> {
            m_shooterAzimuth.setParadoxControl(m_positionRequest.withPosition(0.1 * ShooterConstants.k_shooterGearReduction)); 
        }, this);
    }

    /**
     * Calculates the target rotation for the turret in rotations
     * @return the turret rotation that reaches the target
     */
    private double getTargetRotation(){ 
        /**
         * Position of the robot's turret
         * Created through the Shoot On the Move position and is offset by the turret's offset. Done through a transform3d in order to keep it's offset in the correct rotation if the robot is rotated
         */
        Pose3d hubPose = new Pose3d(Subsystems.m_swerveSubsystem.getPose());
        var rotatedPose = new Pose3d(ShooterConstants.k_turretOffset, new Rotation3d()).rotateBy(new Rotation3d(0, 0, hubPose.getRotation().getAngle()));
        hubPose.plus(new Transform3d(hubPose.getX() + rotatedPose.getX(), hubPose.getY() + rotatedPose.getY(), hubPose.getZ() + rotatedPose.getZ(), hubPose.getRotation()));



        var hub = SwerveSubsystem.isRedAlliance() ? FieldConstants.k_redHub : FieldConstants.k_blueDepot;
        /**
         * This is the X and Y component of the triangle constructed from the robot to the hub
         */
        var rotationTriangle = new Translation2d(hubPose.getX() - hub.getX(), hubPose.getY() - hub.getY());
        /**
         * Uses arctangent to calculate the angle in radians
         */
        // var targetRotation = Math.tan(rotationTriangle.getY() / rotationTriangle.getX());
        var targetRotation = PhotonUtils.getYawToPose(hubPose.toPose2d(), new Pose2d(hub, new Rotation2d()));
        // Checks the 3 coterminal angles to see if any of them are closer to our current position
        // double bestRotation = targetRotation;
        // // checks the rotation with -360, 0, and 360 added to it.
        // for (int i = -1; i <= 1; i++) {
        //     bestRotation = (checkBestRotation(bestRotation, targetRotation + (i * (2 * Math.PI))) * ShooterConstants.k_shooterGearReduction);
        // }
        // System.out.println(targetRotation * ShooterConstants.k_shooterGearReduction);
        return Units.radiansToRotations(targetRotation.getRadians() * ShooterConstants.k_shooterGearReduction);
    }
    /**
     * checks the coterminal angles and finds whihc one is closer to our current position
     * @param bestRotation The current closest rotation
     * @param testRotation The rotation to be tested and compared to the current best rotation
     * @param hubPose The position of the robot
     * @return
     */
    private double checkBestRotation(double bestRotation, double testRotation){
    
        var turretRotation = Units.rotationsToRadians(m_shooterAzimuth.getRotorPosition().getValueAsDouble() * ShooterConstants.k_shooterGearReduction);
        // Checking if the rotation is between our turret's bounds
        if (DoubleNumber.isBetween(testRotation, ShooterConstants.k_turretBounds)){
            // Checking if the distance between our target rotation and our turret's rotation is closer than with the previous best rotation
            if ((testRotation - turretRotation) < (bestRotation - turretRotation)){
                // if it is, that becomes our new best rotation
                bestRotation = testRotation;
            }
        }
        
        return bestRotation;

    }

    public TalonFX getShooterAzimuth(){
        return m_shooterAzimuth;

    }

    public void setShutoff(boolean shutoff) {
        m_shooterAzimuth.setShutoff(shutoff);
    }

    public boolean getShutoff() {
        return m_shooterAzimuth.getShutoff();
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