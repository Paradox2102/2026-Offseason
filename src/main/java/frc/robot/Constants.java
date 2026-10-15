package frc.robot;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerConveyorSubsystem;
import frc.robot.subsystems.indexer.IndexerRollerSubsystem;
import frc.robot.subsystems.intake.IntakeExtensionSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.lights.LightSubsystem;
import frc.robot.subsystems.shooter.SecondaryKicker;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.TurretAimSubsystem;
import frc.robot.utils.DoubleNumber;

public class Constants {


    public static class CANIDConstants {

        public static final CANBus k_canBus0 = new CANBus(CANPort.CAN_S0);
        public static final CANBus k_canBus1 = new CANBus(CANPort.CAN_S1);
        public static final CANBus k_canBus2 = new CANBus(CANPort.CAN_S2);
        public static final CANBus k_canBus3 = new CANBus(CANPort.CAN_S3);
        public static final CANBus k_canBus4 = new CANBus(CANPort.CAN_S4);

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
        public static final int goUp = 45;

    }    

    public static class Subsystems {
        public static final SwerveSubsystem m_swerveSubsystem;
        public static final IntakeExtensionSubsystem m_intakeExtension = new IntakeExtensionSubsystem();
        public static final IntakeRollerSubsystem m_intakeRoller = new IntakeRollerSubsystem();
        public static final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
        public static final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
        public static final TurretAimSubsystem m_turretAimSubsystem = new TurretAimSubsystem();
        public static final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
        public static final SecondaryKicker m_secondaryKicker = new SecondaryKicker();
        public static final IndexerConveyorSubsystem m_indexerConveyor = new IndexerConveyorSubsystem();
        public static final IndexerRollerSubsystem m_indexerRollerSubsystem = new IndexerRollerSubsystem();
        public static final LightSubsystem m_lightSubsystem = new LightSubsystem();
        static {
            m_swerveSubsystem = TunerConstants.createDrivetrain();
        }
    }

    public static class VisionConstants {

        public static final Translation3d frontRightPosition = new Translation3d(Units.inchesToMeters(-12.2212), Units.inchesToMeters(12.6202), Units.inchesToMeters(12.3281));
        public static final Rotation3d frontRightRotation = new Rotation3d(25, -2.012, 148);
        public static final Translation3d sideLeftPosition = new Translation3d(Units.inchesToMeters(-11.0169), Units.inchesToMeters(-13.0384), Units.inchesToMeters(11.9732));
        public static final Rotation3d sideLeftRotation = new Rotation3d(25, 11.148, 65);
        //public static final Translation3d sideLeftPosition = new Translation3d(Units.inchesToMeters(-3.8793),Units.inchesToMeters(14.0335), Units.inchesToMeters(5.3676));
        //public static final Rotation3d sideLeftRotation = new Rotation3d();
        public static final Translation3d frontLeftPos = new Translation3d(Units.inchesToMeters(-12.1548), Units.inchesToMeters(-8.6186), Units.inchesToMeters(12.00));
        public static final Rotation3d frontLeftRot = new Rotation3d(25, 1.823, 147.168);

    }

    public static class LightConstants {
    public static final int k_lightPort = 0; // may be different on actual robot
    public static final int k_lightAmount = 100;

    public static LinearVelocity k_disabledVelocity = MetersPerSecond.of(0.2);
    public static Distance k_ledSpacing = Meters.of(1 / 120.0);
    
  }
  public static class ShooterConstants {
    public static final double k_shooterGearReduction = (67.86);
    
    public static final double k_kickInRPS = 30;
    public static final double k_kickOutRPS = 20;

    public static final double k_secondaryKickerRPS = 30;

    public static final double k_upKickerMOI = 0.00000494 * 2;
    public static final double k_kickerMOI = 0.0000356;
    public static final double k_shooterMOI = 0.0004812;

    public static final TalonFXConfiguration k_hoodConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_shooterConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_turretConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_kickerConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_secondaryKickerConfig = new TalonFXConfiguration();

    public static final double k_hoodG = 0.0;
    public static final double k_hoodS = 0;
    public static final double k_hoodP = 0.1;
    public static final double k_hoodD = 0;

    public static final double k_shooterP = 0.3;
    public static final double k_shooterS = 0.01;
    public static final double k_shooterV = 0.11966;

    public static final double k_kickerP = 0.01;
    public static final double k_kickerS = 0.01;
    public static final double k_kickerV = 0.118;

    public static final double k_secondaryKickerP = 0.0;
    public static final double k_secondaryKickerS = 0.01;
    public static final double k_secondaryKickerV = 0.1195;

    public static final double k_turretI = 0.0;
    public static final double k_turretP = 0.0;
    public static final double k_turretD = 0.0;
    public static final double k_turretV = 0.0;
    public static final double k_turretS = 0.0;
    
    public static final Translation3d k_turretOffset = new Translation3d(Units.inchesToMeters(-5.965), Units.inchesToMeters(7.5429), Units.inchesToMeters(10.3851));

    public static final DoubleNumber k_turretBounds = new DoubleNumber(-250, 250);
    public static final double[][] k_scoreHoodAngle = {
        {1.125, 0},
      {1.25, 0},
      {1.5, 10},
      {1.75, 12.5},
      {2, 15},
      {2.25, 17.5},
      {2.5, 19},
      {2.75, 21},
      {3, 22},
      {4, 22},
      {4.5, 23},
      {5, 25},
      {10, 35}
    };

    public static final double[][] k_scoreShotSpeeds = {
        {1.125, 40},
      {2.25, 45},
      {2.75, 47},
      {3.25, 50},
      {4, 55},
      {4.75, 60}
    };

    public static final double[][] k_scoreShotTimes = {
        {0, 0}
    };
    public static final double[][] k_passHoodAngle = {
      {1.125, 0},
      {1.25, 0},
      {1.5, 10},
      {1.75, 12.5},
      {2, 15},
      {2.25, 17.5},
      {2.5, 19},
      {2.75, 21},
      {3, 22},
      {4, 22},
      {4.5, 23},
      {5, 25},
      {10, 35}
    };

    public static final double[][] k_passShotSpeeds = {
        {1.125, 40},
      {2.25, 45},
      {2.75, 47},
      {3.25, 50},
      {4, 55},
      {4.75, 60}
    };

    public static final double[][] k_passShotTimes = {
        {0, 0}
    };

    public static final double k_staticShotSpeed = 40;
    public static final double k_staticShotAngle = 20;
    public static final double k_hoodMOI = 0.02635296; 
    public static final double k_hoodArmLength = 0.20860766;
    public static final double k_hoodMaxAngle = 90;
    
    public static final double k_hoodSupplyLimit = 26;
    public static final double k_kickerSupplyLimit = 23;
    public static final double k_secondaryKickerSupplyLimit = 23;
    public static final double k_shooterSupplyLimit = 37;
    public static final double k_aimingSupplyLimit = 29;

    public static final CurrentLimitsConfigs k_hoodLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_hoodSupplyLimit);
    public static final CurrentLimitsConfigs k_kickerLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_kickerSupplyLimit);
    public static final CurrentLimitsConfigs k_secondaryKickerLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_secondaryKickerSupplyLimit);
    public static final CurrentLimitsConfigs k_aimingLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_aimingSupplyLimit);
    public static final CurrentLimitsConfigs k_shooterLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_shooterSupplyLimit);

    static {
        k_shooterConfigs.withCurrentLimits(k_shooterLimitConfig);
        k_shooterConfigs.Slot0.kP = k_shooterP;
        k_shooterConfigs.Slot0.kS = k_shooterS;
        k_shooterConfigs.Slot0.kV = k_shooterV;

        k_kickerConfig.withCurrentLimits(k_kickerLimitConfig);
        k_kickerConfig.Slot0.kP = k_kickerP;
        k_kickerConfig.Slot0.kS = k_kickerS;
        k_kickerConfig.Slot0.kV = k_kickerV;

        k_secondaryKickerConfig.withMotorOutput(k_secondaryKickerConfig.MotorOutput.clone().withInverted(InvertedValue.Clockwise_Positive)).withCurrentLimits(k_secondaryKickerLimitConfig);
        k_secondaryKickerConfig.Slot0.kP = k_secondaryKickerP;
        k_secondaryKickerConfig.Slot0.kS = k_secondaryKickerS;
        k_secondaryKickerConfig.Slot0.kV = k_secondaryKickerV;

        k_turretConfig.withCurrentLimits(k_aimingLimitConfig);
        //k_turretConfig.Slot0.kV = k_turretV;
        //k_turretConfig.Slot0.kI = k_turretI;
        k_turretConfig.Slot0.kP = k_turretP;
        k_turretConfig.Slot0.kD = k_turretD;
        k_turretConfig.Slot0.kS = k_turretS;

        k_hoodConfig.withCurrentLimits(k_hoodLimitConfig);
        k_hoodConfig.withMotorOutput(k_hoodConfig.MotorOutput.clone().withInverted(InvertedValue.CounterClockwise_Positive));
        // k_hoodConfig.Slot0.kG = k_hoodG;
        // k_hoodConfig.Slot0.kS = k_hoodS;
        k_hoodConfig.Slot0.kP = k_hoodP;
        // k_hoodConfig.Slot0.kD = k_hoodD;
    }
  }

  public static class FieldConstants {
    public static final Translation2d k_blueHub = new Translation2d(4.62, 4.03);
    public static final Translation2d k_redHub = new Translation2d(11.92, 4.03);

    public static Translation2d k_blueOutpost = new Translation2d(1, 1);
    public static Translation2d k_blueDepot = new Translation2d(1, 7);

    public static Translation2d k_redOutpost = new Translation2d(15.6, 1);
    public static Translation2d k_redDepot = new Translation2d(15.6, 7);

    public static Translation2d k_fullPassRedDepot = new Translation2d(11.3, 5.5); // Need to implement this
    public static Translation2d k_fullPassRedOutpost = new Translation2d(11.3, 2.5);
    public static Translation2d k_fullPassBlueDepot = new Translation2d(5.3, 5.5);
    public static Translation2d k_fullPassBlueOutpost = new Translation2d(5.3, 2.5);

    public static final double k_redZoneTransition = 11.925;
    public static final double k_blueZoneTransition = 4.625;
    public static final double k_yCenterline = 4.0;
    public static final double k_depotIntermediatePoint = 5.940;
    public static final double k_outpostIntermediatePoint = 1.0;
    
    // public static final Rectangle2d k_blueTrenchLeft = new Rectangle2d(
    //     new Translation2d(2.5, 8),
    //     new Translation2d(6.6, 6.8)
    // );

    // public static final Rectangle2d k_blueTrenchRight = new Rectangle2d(
    //     new Translation2d(2.5, 1.3),
    //     new Translation2d(6.6, 0)
    // );

    // public static final Rectangle2d k_redTrenchLeft = new Rectangle2d(
    //     new Translation2d(14, 8),
    //     new Translation2d(10.2, 6.8)
    // );

    // public static final Rectangle2d k_redTrenchRight = new Rectangle2d(
    //     new Translation2d(14, 1.3),
    //     new Translation2d(10.2, 0)
    // );
  }

  

  public static class IntakeConstants {
    public static final double k_inRPS = 40;
    public static final double k_outRPS = 30;

    public static final double k_outPosition = 90;

    public static final TalonFXConfiguration k_intakeRollerConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_intakeExtensionConfigs = new TalonFXConfiguration();

    public static final double k_rollerP = 0.1;
    public static final double k_rollerS = 0.0;
    public static final double k_rollerV = 0.0;

    public static final double k_intakeExtensionG = 0;
    public static final double k_intakeExtensionP = 0.1;
    public static final double k_intakeExtensionV = 0;
    public static final double k_intakeExtensionD = 0;

    public static final double k_extensionSupplyLimit = 30;
    public static final double k_rollerSupplyLimit = 30;

    public static final CurrentLimitsConfigs k_extensionLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_extensionSupplyLimit);
    public static final CurrentLimitsConfigs k_rollerLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_rollerSupplyLimit);

    
    public static final double k_intakeRollerMOI = 0.093084;
    static {
        // k_intakeRollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        k_intakeRollerConfigs.withMotorOutput(k_intakeRollerConfigs.MotorOutput.clone().withInverted(InvertedValue.Clockwise_Positive)).withCurrentLimits(k_rollerLimitConfig);
        k_intakeRollerConfigs.Slot0.kP = k_rollerP;
        k_intakeRollerConfigs.Slot0.kV = k_rollerV;
        k_intakeRollerConfigs.Slot0.kS = k_rollerS;

        k_intakeExtensionConfigs.withCurrentLimits(k_extensionLimitConfig);
        k_intakeExtensionConfigs.Slot0.kG = k_intakeExtensionG;
        k_intakeExtensionConfigs.Slot0.kP = k_intakeExtensionP;
        k_intakeExtensionConfigs.Slot0.kV = k_intakeExtensionV;
        k_intakeExtensionConfigs.Slot0.kD = k_intakeExtensionD;
    }

    
    }
    public static class IndexerConstants {
        public static final double k_rollerInRPS = 50;
        public static final double k_rollerOutRPS = -40;

        public static final double k_conveyorInRPS = 20;
        public static final double k_conveyorOutRPS = -40;

        public static final int k_jamThreshold = 6; // should be tuned to not happen too much

        public static final TalonFXConfiguration m_indexerRollerConfigs = new TalonFXConfiguration();
        public static final TalonFXConfiguration m_indexerConveyorConfigs = new TalonFXConfiguration();
        public static final double k_indexerRollerMOI = 0.00045822;
        public static final double k_indexerConveyorMOI = 0.00001722;

        public static final double k_rollerP = 0.3;
        public static final double k_rollerS = 0.0;
        public static final double k_rollerV = 0.125;

        public static final double k_conveyorP = 0.0;
        public static final double k_conveyorS = 0.1;
        public static final double k_conveyorV = 0.0;

        public static final double k_conveyorMotionMagicAcceleration = 0.1;
        public static final double k_rollerMotionMagicAcceleration = 300;

        public static final double k_conveyorSupplyLimit = 18;
    public static final double k_rollerSupplyLimit = 25;

    public static final CurrentLimitsConfigs m_conveyorLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_conveyorSupplyLimit);
    public static final CurrentLimitsConfigs m_rollerLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_rollerSupplyLimit);

    public static final MotionMagicConfigs m_indexerConveyorMotionMagicConfig = m_indexerConveyorConfigs.MotionMagic.withMotionMagicAcceleration(k_conveyorMotionMagicAcceleration);
    public static final MotionMagicConfigs m_indexerRollerMotionMagicConfig = m_indexerRollerConfigs.MotionMagic.withMotionMagicAcceleration(k_rollerMotionMagicAcceleration);

        static {

            // m_indexerRollerMotionMagicConfig.MotionMagicAcceleration = k_rollerMotionMagicAcceleration;

            m_indexerRollerConfigs.withCurrentLimits(m_rollerLimitConfig);
            m_indexerRollerConfigs.Slot0.kP = k_rollerP;
            m_indexerRollerConfigs.Slot0.kS = k_rollerS;
            m_indexerRollerConfigs.Slot0.kV = k_rollerV;

            
            // m_indexerConveyorMotionMagicConfig.withMotionMagicAcceleration(k_rollerMotionMagicAcceleration);
            
            m_indexerConveyorConfigs.withCurrentLimits(m_rollerLimitConfig);
            m_indexerConveyorConfigs.Slot0.kP = k_conveyorP;
            m_indexerConveyorConfigs.Slot0.kS = k_conveyorS;
            m_indexerConveyorConfigs.Slot0.kV = k_conveyorV;
            
        }
  }
}
