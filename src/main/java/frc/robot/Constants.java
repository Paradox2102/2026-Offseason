package frc.robot;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Units;

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

        // The PDH is alone on CAN_S0 with CAN ID 21 (REV default is 1)
        public static final CANPort k_pdhPort = CANPort.CAN_S0;
        public static final int pdh = 21;

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
        public static final ShooterSubsystem m_shooterSubsystem;
        public static final HoodSubsystem m_hoodSubsystem;
        public static final TurretAimSubsystem m_turretAimSubsystem = new TurretAimSubsystem();
        public static final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
        public static final SecondaryKicker m_secondaryKicker = new SecondaryKicker();
        public static final IndexerConveyorSubsystem m_indexerConveyor = new IndexerConveyorSubsystem();
        public static final IndexerRollerSubsystem m_indexerRollerSubsystem = new IndexerRollerSubsystem();
        public static final LightSubsystem m_lightSubsystem = new LightSubsystem();
        static {
            m_swerveSubsystem = TunerConstants.createDrivetrain();
            m_hoodSubsystem  = new HoodSubsystem(Subsystems.m_turretAimSubsystem::getTargetDistance);
            m_shooterSubsystem = new ShooterSubsystem(Subsystems.m_turretAimSubsystem::getTargetDistance);
        }
    }

    public static class DrivebaseConstants {
        //Wheel Calibration
    public static final double k_wheelRadiusMaxVelocity = 0.5; // Rad/Sec
    public static final double k_wheelRadiusRampRate = 0.25; // Rad/Sec^2

    // Distance from the robot center to a wheel, in meters. Read it from the
    // swerve module positions so there is only one place to change it.
    public static final double k_driveBaseRadius = Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);
    }

    /**
     * Where each camera sits on the robot, from the CAD (Onshape "G0000 - Main", 2026-10-19).
     *
     * WPILib robot frame: +x = toward the intake, +y = LEFT, +z = UP, origin on the floor
     * at the robot center. A CAD export (glTF) is Y-up: there the second number is the height.
     * Always change CAD numbers into this frame before they go here.
     *
     * Angles are Rotation3d(roll, pitch, yaw) of the lens axis: a NEGATIVE pitch tilts the
     * camera UP, a positive yaw turns it to the left. The camera looks along its own +x.
     */
    public static class VisionConstants {

		        // CAD cambrea 1: back left corner, looks back and right
        public static final Rotation3d backLeftRotation = camRotation(-17.3, -30.0, -148.0);
        public static final Translation3d backLeftPosition = camPosition(-11.53, 10.05, 8.69, backLeftRotation);
        // CAD cambrea 2: back right corner, looks back and left
        public static final Rotation3d backRightRotation = camRotation(-2.1, -25.0, 148.0);
        public static final Translation3d backRightPosition = camPosition(-11.44, -9.09, 16.22, backRightRotation);
        // CAD cambrea 4: left side, looks left
        public static final Rotation3d sideLeftRotation = camRotation(-8.9, -30.0, 65.0);
        public static final Translation3d sideLeftPosition = camPosition(-0.80, 13.01, 9.13, sideLeftRotation);
        // CAD cambrea 3: right side, looks right
        public static final Rotation3d sideRightRotation = camRotation(-11.1, -25.0, -65.0);
        public static final Translation3d sideRightPosition = camPosition(-11.43, -12.73, 16.18, sideRightRotation);

        /** CAD angles are in degrees; Rotation3d wants radians. */
        private static Rotation3d camRotation(double rollDeg, double pitchDeg, double yawDeg) {
            return new Rotation3d(Units.degreesToRadians(rollDeg), Units.degreesToRadians(pitchDeg), Units.degreesToRadians(yawDeg));
        }

        /**
         * The CAD numbers (inches) are the camera part's origin, 0.98 in behind the lens front.
         * Move half of that along the lens axis, close to where the lens takes its picture from.
         */
        private static Translation3d camPosition(double xIn, double yIn, double zIn, Rotation3d rotation) {
            Translation3d alongLens = new Translation3d(Units.inchesToMeters(0.49), 0, 0).rotateBy(rotation);
            return new Translation3d(Units.inchesToMeters(xIn), Units.inchesToMeters(yIn), Units.inchesToMeters(zIn)).plus(alongLens);
        }


        public static final Vector<N3> k_visionBaseSDev = VecBuilder.fill(15, 15, 15);
        public static final Vector<N3> k_multiTagSDev = VecBuilder.fill(.5, .5, .5);

        public static final int[] ignoreTagList = {};
    }


  public static class ShooterConstants {
    public static final double k_shooterGearReduction = (67.86);
    
    public static final double k_kickOutRPS = -90;
    public static final double k_kickSlowOutRPS = -40;
    
    public static final double k_secondaryKickerInRPS = 90;
    public static final double k_secondaryKickerOutRPS = -30;

public static final double k_secondarySlowKickerInRPS = 20;
    public static final double k_secondarySlowKickerOutRPS = -20;

    public static final double k_upKickerMOI = 0.00000494 * 2;
    public static final double k_kickerMOI = 0.0000356;
    public static final double k_shooterMOI = 0.0004812;

    public static final TalonFXConfiguration k_hoodConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_shooterConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_turretConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_kickerConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_secondaryKickerConfig = new TalonFXConfiguration();

    public static final double k_hoodG = 0.0;
    public static final double k_hoodS = 1.2;
    public static final double k_hoodP = 50.0;
    public static final double k_hoodD = 0.5;

    public static final double k_shooterP = 0.008;
    public static final double k_shooterS = 0.33;
    public static final double k_shooterV = 0.082;
    public static final double k_shooterI = 0.0;
    public static final double k_shooterD = 0.0;

    public static final double k_kickerP = 0.09;
    public static final double k_kickerS = 0.25;
    public static final double k_kickerV = 0.118;

    public static final double k_secondaryKickerP = 0.06;
    public static final double k_secondaryKickerS = 0.6;
    public static final double k_secondaryKickerV = 0.115;
    public static final double k_secondaryKickerI = 0.03;

    public static final double k_turretI = 0.0;
    public static final double k_turretP = 2.0;
    public static final double k_turretD = 0.0;
    public static final double k_turretV = 0.0;
    public static final double k_turretS = 0.0;
    
    public static final Translation2d k_turretOffset = new Translation2d(-0.151554, 0.190500);

    public static final DoubleNumber k_turretBounds = new DoubleNumber(-250, 250);

    /** Time from the pose being measured to the fuel leaving the shooter in seconds */
    public static final double k_sotmLatency = 1.00; // PLEASE TUNE ME
    /** How much each operator latency button press changes the latency, seconds */
    public static final double k_sotmLatencyStep = 0.01;
    /** Weird math stuff. Stolen from 4322 who stole it from 5962 */
    public static final double k_sotmDragCoeff = 0.24;
    
    public static final double k_sotmMinSpeed = 0.1;
    public static final double k_sotmMaxSpeed = 100;
    public static final double[][] k_scoreHoodAngle = {
        {1.26, 0},
        {1.7, 0},
        {2.2, 0.6},
        {2.7, 0.8},
        {3.2, 1.2},
        {3.7, 2.2},
        {4.2, 3.0},
        {4.7, 4.0},
        {5.2, 4.5},
        {5.5, 5.7}
    };

    public static final double[][] k_scoreShotSpeeds = {
        {1.26, 58},
        {1.7, 64},
        {2.2, 65},
        {2.7, 67},
        {3.2, 70},
        {3.7, 72},
        {4.2, 76},
        {4.7, 78},
        {5.2, 82},
        {5.5, 85}
    };

    public static final double[][] k_scoreShotTimes = {
        {0.8, 0}
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
    public static final double k_staticShotAngle = 6.0;
    public static final double k_hoodMOI = 0.02635296; 
    public static final double k_hoodArmLength = 0.20860766;
    public static final double k_hoodMaxAngle = 8.310546875;
    
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

    public static final double k_kickerAcceleration = 40.0;
    public static final double k_secondaryKickerAcceleration = 40.0;

    public static final double k_shooterFlywheelAcceleration = 60.0;

    public static final MotionMagicConfigs m_shooterMotionMagicConfig = k_shooterConfigs.MotionMagic.withMotionMagicAcceleration(k_shooterFlywheelAcceleration);
    public static final MotionMagicConfigs m_kickerMotionMagicConfig = k_kickerConfig.MotionMagic.withMotionMagicAcceleration(k_kickerAcceleration);
    public static final MotionMagicConfigs m_secondaryKickerMotionMagicConfig = k_secondaryKickerConfig.MotionMagic.withMotionMagicAcceleration(k_secondaryKickerAcceleration);


    static {
        k_shooterConfigs.withCurrentLimits(k_shooterLimitConfig).withMotorOutput(k_shooterConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
        k_shooterConfigs.Slot0.kP = k_shooterP;
        k_shooterConfigs.Slot0.kS = k_shooterS;
        k_shooterConfigs.Slot0.kV = k_shooterV;
        k_shooterConfigs.Slot0.kI = k_shooterI;
        k_shooterConfigs.Slot0.kD = k_shooterD;

        k_kickerConfig.withCurrentLimits(k_kickerLimitConfig);
        k_kickerConfig.Slot0.kP = k_kickerP;
        k_kickerConfig.Slot0.kS = k_kickerS;
        k_kickerConfig.Slot0.kV = k_kickerV;

        k_secondaryKickerConfig.withMotorOutput(k_secondaryKickerConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)).withCurrentLimits(k_secondaryKickerLimitConfig);
        k_secondaryKickerConfig.Slot0.kP = k_secondaryKickerP;
        k_secondaryKickerConfig.Slot0.kS = k_secondaryKickerS;
        k_secondaryKickerConfig.Slot0.kV = k_secondaryKickerV;
        k_secondaryKickerConfig.Slot0.kI = k_secondaryKickerI;

        k_turretConfig.withCurrentLimits(k_aimingLimitConfig);
        //k_turretConfig.Slot0.kV = k_turretV;
        //k_turretConfig.Slot0.kI = k_turretI;
        k_turretConfig.Slot0.kP = k_turretP;
        k_turretConfig.Slot0.kD = k_turretD;
        // k_turretConfig.Slot0.kS = k_turretS;

        k_hoodConfig.withCurrentLimits(k_hoodLimitConfig);
        k_hoodConfig.withMotorOutput(k_hoodConfig.MotorOutput.clone().withInverted(InvertedValue.CounterClockwise_Positive)).withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs().withForwardSoftLimitThreshold(k_hoodMaxAngle).withForwardSoftLimitEnable(true).withReverseSoftLimitThreshold(0).withReverseSoftLimitEnable(true));
        // k_hoodConfig.Slot0.kG = k_hoodG;
        k_hoodConfig.Slot0.kS = k_hoodS;
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
    public static final double k_inRPS = 90;
    public static final double k_outRPS = -70;

    public static final TalonFXConfiguration m_intakeRollerConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration m_intakeExtensionConfigs = new TalonFXConfiguration();

    public static final double k_rollerP = 0.0;
    public static final double k_rollerS = 0.0;
    public static final double k_rollerV = 0.12;

    public static final double k_intakeExtensionG = 0;
    public static final double k_intakeExtensionP = 0.1;
    public static final double k_intakeExtensionV = 1.5;
    public static final double k_intakeExtensionD = 0;

    public static final double k_extensionSupplyLimit = 30;
    public static final double k_rollerSupplyLimit = 30;

    public static final CurrentLimitsConfigs k_extensionLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_extensionSupplyLimit);
    public static final CurrentLimitsConfigs k_rollerLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_rollerSupplyLimit);

    public static final double m_intakeExtensionAcceleration = 20.0;
    public static final double m_rollerMotionMagicAcceleration = 40.0;

    public static final MotionMagicConfigs m_intakeExtensionMotionMagicConfig = m_intakeExtensionConfigs.MotionMagic.withMotionMagicAcceleration(m_intakeExtensionAcceleration);
    public static final MotionMagicConfigs m_intakeRollerMotionMagicConfig = m_intakeRollerConfigs.MotionMagic.withMotionMagicAcceleration(m_rollerMotionMagicAcceleration);

    
    public static final double k_intakeRollerMOI = 0.093084;
    static {
        // k_intakeRollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_intakeRollerConfigs.withMotorOutput(m_intakeRollerConfigs.MotorOutput.clone().withInverted(InvertedValue.Clockwise_Positive));
        m_intakeRollerConfigs.withCurrentLimits(k_rollerLimitConfig);

        m_intakeRollerConfigs.MotionMagic = m_intakeRollerMotionMagicConfig;

        m_intakeRollerConfigs.Slot0.kP = k_rollerP;
        m_intakeRollerConfigs.Slot0.kV = k_rollerV;
        m_intakeRollerConfigs.Slot0.kS = k_rollerS;

        m_intakeExtensionConfigs.MotionMagic = m_intakeExtensionMotionMagicConfig;

        m_intakeExtensionConfigs.withCurrentLimits(k_extensionLimitConfig);
        m_intakeExtensionConfigs.Slot0.kG = k_intakeExtensionG;
        m_intakeExtensionConfigs.Slot0.kP = k_intakeExtensionP;
        m_intakeExtensionConfigs.Slot0.kV = k_intakeExtensionV;
        m_intakeExtensionConfigs.Slot0.kD = k_intakeExtensionD;
    }

    
    }
    public static class IndexerConstants {
        public static final double k_rollerInRPS = 40;
        public static final double k_rollerOutRPS = -40;
        public static final double k_rollerSlowInRPS = 20;
        public static final double k_rollerSlowOutRPS = -20;

        public static final double k_conveyorInRPS = 90;
        public static final double k_conveyorOutRPS = -70;
        public static final double k_conveyorSlowInRPS = 30;
        public static final double k_conveyorSlowOutRPS = -20;

        public static final int k_jamThreshold = 6; // should be tuned to not happen too much

        public static final TalonFXConfiguration m_indexerRollerConfigs = new TalonFXConfiguration();
        public static final TalonFXConfiguration m_indexerConveyorConfigs = new TalonFXConfiguration();
        public static final double k_indexerRollerMOI = 0.00045822;
        public static final double k_indexerConveyorMOI = 0.00001722;

        public static final double k_rollerP = 0.15;
        public static final double k_rollerS = 0.0;
        public static final double k_rollerV = 0.254;
        public static final double k_rollerI = 0.2;

        public static final double k_conveyorP = 0.1;
        public static final double k_conveyorI = 0.2;

        public static final double k_conveyorV = 0.097;

        public static final double k_conveyorMotionMagicAcceleration = 40.0;
        public static final double k_rollerMotionMagicAcceleration = 40.0;

        public static final double k_conveyorSupplyLimit = 25;
        public static final double k_rollerSupplyLimit = 25;

        public static final CurrentLimitsConfigs m_conveyorLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_conveyorSupplyLimit);
        public static final CurrentLimitsConfigs m_rollerLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(k_rollerSupplyLimit);

        public static final MotionMagicConfigs m_indexerConveyorMotionMagicConfig = m_indexerConveyorConfigs.MotionMagic.withMotionMagicAcceleration(k_conveyorMotionMagicAcceleration);
        public static final MotionMagicConfigs m_indexerRollerMotionMagicConfig = m_indexerRollerConfigs.MotionMagic.withMotionMagicAcceleration(k_rollerMotionMagicAcceleration);

        static {

            m_indexerRollerMotionMagicConfig.MotionMagicAcceleration = k_rollerMotionMagicAcceleration;

            m_indexerRollerConfigs.withCurrentLimits(m_rollerLimitConfig).withMotorOutput(m_indexerRollerConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
            m_indexerRollerConfigs.Slot0.kP = k_rollerP;
            m_indexerRollerConfigs.Slot0.kS = k_rollerS;
            m_indexerRollerConfigs.Slot0.kV = k_rollerV;
            m_indexerRollerConfigs.Slot0.kI = k_rollerI;

            
            m_indexerConveyorMotionMagicConfig.withMotionMagicAcceleration(k_conveyorMotionMagicAcceleration);
            
            m_indexerConveyorConfigs.withCurrentLimits(m_conveyorLimitConfig);
            m_indexerConveyorConfigs.Slot0.kP = k_conveyorP;
            m_indexerConveyorConfigs.Slot0.kI = k_conveyorI;
            m_indexerConveyorConfigs.Slot0.kV = k_conveyorV;
            
        }
  }

    /** Stall shutoff for every ParadoxTalon; see utils/ParadoxTalon.java */
    public static class StallConstants {
        /** Above this stator current while barely moving counts as a stall. */
        public static final double k_stallCurrentAmps = 40;
        public static final double k_stallVelocityRps = 0.5;
        /** How long a stalled motor stays off before it tries again. */
        public static final double k_cooldownSeconds = 2.0;
        /** A PositionVoltage motor this close to its target is holding, not stalled. */
        public static final double k_atSetpointToleranceRotations = 0.02;
    }
}
