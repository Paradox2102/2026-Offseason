package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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

        public static final int indexerRoller = 20;
        public static final int indexerConveyorLeader = 21;
        public static final int indexerConveyorFollower = 22;

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

  public static class IntakeConstants {
    public static final int k_inRPM = 5000;
    public static final int k_outRPM = -4000;

    public static final Slot0Configs k_intakeRollerConfigs = new Slot0Configs();
    static {
        k_intakeRollerConfigs.kS = 0.1; // Add 0.1 V output to overcome static friction
        k_intakeRollerConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        k_intakeRollerConfigs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        k_intakeRollerConfigs.kI = 0; // no output for integrated error
        k_intakeRollerConfigs.kD = 0; // no output for error derivative
    }
    
  }

  public static class Globals {
    public static void setRPM(TalonFX motor, VelocityVoltage request, double RPM){
        motor.setControl(request.withFeedForward(RPM));
    }
  }
}
