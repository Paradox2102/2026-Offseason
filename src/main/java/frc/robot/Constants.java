package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class Constants {
    public static class CANIDConstants {
        public static final int gyro = 0;

        public static final int fl_drive = 1;
        public static final int fl_steer = 2;
        public static final int fr_drive = 3;
        public static final int fr_steer = 4;
        public static final int bl_drive = 5;
        public static final int bl_steer = 6;
        public static final int br_drive = 7;
        public static final int br_steer = 8;
        public static final int fl_encoder = 9;
        public static final int fr_encoder = 10;
        public static final int bl_encoder = 11;
        public static final int br_encoder = 12;

        public static final int intakeRoller = 20;
        public static final int intakeExtensionLeader = 21;
        public static final int intakeExtensionFollower = 22;

        public static final int indexerRoller = 30;
        public static final int indexerConveyorLeader = 31;
        public static final int indexerConveyorFollower = 32;

        public static final int kicker = 40;
        public static final int hood = 41;
        public static final int shooterLeader = 42;
        public static final int shooterFollower = 43;
        public static final int shooterAzimuth = 44;


    }    

    public static class Subsystems {
        public static final SwerveSubsystem m_swerveSubsystem = TunerConstants.createDrivetrain();
    }

    public static class VisionConstants {

        public static final Translation3d frontLeftPosition = new Translation3d();
        public static final Rotation3d frontLeftRotation = new Rotation3d();
        public static final Translation3d frontRightPosition = new Translation3d();
        public static final Rotation3d frontRightRotation = new Rotation3d();
        public static final Translation3d sideLeftPosition = new Translation3d();
        public static final Rotation3d sideLeftRotation = new Rotation3d();
        public static final Translation3d sideRightPosition = new Translation3d();
        public static final Rotation3d sideRightRotation = new Rotation3d();

    }

    public static class LightConstants {
    public static final int k_lightPort = 0; // may be different on actual robot
    public static final int k_lightAmount = 100;

    public static LinearVelocity k_disabledVelocity = MetersPerSecond.of(0.2);
    public static Distance k_ledSpacing = Meters.of(1 / 120.0);
    
  }
  public static class ShooterConstants {
    public static final int k_kickInRPM = 5000;
    public static final int k_kickOutRPM = 1000;

    public static final Slot0Configs k_hoodConfig = new Slot0Configs();
    public static final Slot0Configs k_shooterConfigs = new Slot0Configs();
    public static final Slot0Configs k_turretConfig = new Slot0Configs();

    public static final double k_hoodG = 0;
    public static final double k_hoodS = 0;
    public static final double k_hoodP = 0;
    public static final double k_hoodD = 0;

    public static final double k_shooterP = 0;
    public static final double k_shooterS = 0;
    public static final double k_shooterV = 0;

    public static final double k_turretS = 0;
    public static final double k_turretP = 0;
    public static final double k_turretD = 0;

    public static final double[][] m_hoodAngle = {
        {0, 0}
    };

    public static final double[][] m_shotSpeeds = {
        {0, 0}
    };

    public static final double k_hoodMOI = 0.0254713554;
    public static final double k_hoodArmLength = 0.0254713554;
    public static final double k_hoodMaxAngle = 90;

    static {
        k_shooterConfigs.kP = k_shooterP;
        k_shooterConfigs.kS = k_shooterS;
        k_shooterConfigs.kV = k_shooterV;

        k_turretConfig.kS = k_turretS;
        k_turretConfig.kP = k_turretP;
        k_turretConfig.kD = k_turretD;

        k_hoodConfig.kG = k_hoodG;
        k_hoodConfig.kS = k_hoodS;
        k_hoodConfig.kP = k_hoodP;
        k_hoodConfig.kD = k_hoodD;
    }
  }

  public static class FieldConstants {
    public static final Translation2d k_blueHub = new Translation2d(4.62, 4.03);
    public static final Translation2d k_redHub = new Translation2d(11.92, 4.03);
  }

  

  public static class IntakeConstants {
    public static final int k_inRPM = 5000;
    public static final int k_outRPM = -4000;

    public static final int k_outPosition = 90;

    public static final Slot0Configs k_intakeRollerConfigs = new Slot0Configs();
    public static final Slot0Configs k_intakeExtensionConfigs = new Slot0Configs();

        public static final double k_rollerP = 0;
        public static final double k_rollerS = 0;
        public static final double k_rollerV = 0;

    public static final double k_intakeExtensionG = 0;
    public static final double k_intakeExtensionP = 0;
    public static final double k_intakeExtensionV = 0;
    public static final double k_intakeExtensionD = 0;

    public static final double k_intakeRollerMOI = 7.8368899188;
    static {
        k_intakeRollerConfigs.kP = k_rollerP;
        k_intakeRollerConfigs.kS = k_rollerS;
        k_intakeRollerConfigs.kV = k_rollerV;

        k_intakeExtensionConfigs.kG = k_intakeExtensionG;
        k_intakeExtensionConfigs.kP = k_intakeExtensionP;
        k_intakeExtensionConfigs.kV = k_intakeExtensionV;
        k_intakeExtensionConfigs.kD = k_intakeExtensionD;
    }

    public static class IndexerConstants {
        public static final int k_rollerInRPM = 3000;
        public static final int k_rollerOutRPM = -2750;

        public static final int k_conveyorInRPM = 3000;
        public static final int k_conveyorOutRPM = -2750;


        public static final Slot0Configs k_indexerRollerConfigs = new Slot0Configs();
        public static final Slot0Configs k_indexerConveyorConfigs = new Slot0Configs();

        public static final double k_indexerRollerMOI = 4.2552029673;
        public static final double k_indexerConveyorMOI = 0.0848654995;

        public static final double k_rollerP = 0;
        public static final double k_rollerS = 0;
        public static final double k_rollerV = 0;

        public static final double k_conveyorP = 0;
        public static final double k_conveyorS = 0;
        public static final double k_conveyorV = 0;

        static {
            k_indexerRollerConfigs.kP = k_rollerP;
            k_indexerRollerConfigs.kS = k_rollerS;
            k_indexerRollerConfigs.kV = k_rollerV;

            k_indexerConveyorConfigs.kP = k_conveyorP;
            k_indexerConveyorConfigs.kS = k_conveyorS;
            k_indexerConveyorConfigs.kV = k_conveyorV;
            
        }
    }
    
  }

  public static class Globals {
    /**
     * Sets the RPM of a specified motor
     * @param motor The motor to set the RPM of
     * @param request This is a VelocityVoltage. Should be created once in your class then sed every time you set RPM
     * @param RPM This is the RPM the motor will go spiin
     */
    public static void setRPM(TalonFX motor, VelocityVoltage request, double RPM){
        motor.setControl(request.withFeedForward(RPM));
    }
    /**
     * Sets the RPM of a specified motor
     * @param motor The motor to set the RPM of
     * @param request This is a PositionVoltage. Should be created once in your class then said every time you set RPM
     * @param position This is the position the motor will go spiin
     */
    public static void setPosition(TalonFX motor, PositionVoltage request, double position){
        motor.setControl(request.withPosition(position));
    }
    /**
     * Sets the RPM of a lead and follower motor
     * @param leadMotor The lead motor to have the RPM set for
     * @param followMotor The follower motor to have the RPM of the lead motor
     * @param request This is a VelocityVoltage. Should be created once in your class then said every time you set RPM
     * @param RPM Motor go spiin at rpm
     * @param alignmentValue Whether the motors are aligned or opposed (ugh, I think I mispelled that)
     */
    public static void setFollowerRPM(TalonFX leadMotor, TalonFX followMotor, VelocityVoltage request, double RPM, MotorAlignmentValue alignmentValue){
        setRPM(leadMotor, request, RPM);
        followMotor.setControl(new Follower(leadMotor.getDeviceID(), alignmentValue));
    }

    /**
     * Sets the Position of a lead and follower motor
     * @param leadMotor The lead motor to have the RPM set for
     * @param followMotor The follower motor to have the RPM of the lead motor
     * @param request This is a VelocityVoltage. Should be created once in your class then said every time you set RPM
     * @param position Motor go spiin to position
     * @param alignmentValue Whether the motors are aligned or opposed (ugh, I think I mispelled that)
     */
    public static void setFollowerPosition(TalonFX leadMotor, TalonFX followMotor, PositionVoltage request, double position, MotorAlignmentValue alignmentValue){
        setPosition(leadMotor, request, position);
        followMotor.setControl(new Follower(leadMotor.getDeviceID(), alignmentValue));
    }
  }
}
