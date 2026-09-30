package frc.robot;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import org.wpilib.math.geometry.Rectangle2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import frc.robot.PathPlanningThing.DrivePath;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerConveyorSubsystem;
import frc.robot.subsystems.indexer.IndexerRollerSubsystem;
import frc.robot.subsystems.intake.IntakeExtensionSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.SecondaryKicker;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.TurretAimSubsystem;
import frc.robot.utils.DoubleNumber;

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
        static {
            m_swerveSubsystem = TunerConstants.createDrivetrain();
        }
    }

    public static class VisionConstants {

        public static final Translation3d backLeftPosition = new Translation3d(-12.2212, 12.6202, 12.3281);
        public static final Rotation3d backLeftRotation = new Rotation3d();
        public static final Translation3d backRightPosition = new Translation3d(-11.0169, -13.0384, 11.9732);
        public static final Rotation3d backRightRotation = new Rotation3d();
        public static final Translation3d sideLeftPosition = new Translation3d(-3.8793,14.0335, 5.3676);
        public static final Rotation3d sideLeftRotation = new Rotation3d();
        public static final Translation3d blueButNotInCornerPos = new Translation3d();
        public static final Rotation3d blueButNotInCornerRot = new Rotation3d();

    }

    public static class LightConstants {
    public static final int k_lightPort = 0; // may be different on actual robot
    public static final int k_lightAmount = 100;

    public static LinearVelocity k_disabledVelocity = MetersPerSecond.of(0.2);
    public static Distance k_ledSpacing = Meters.of(1 / 120.0);
    
  }
  public static class ShooterConstants {
    public static final double k_shooterGearReduction = (67.86);
    // for every one 67.86 turns of the motor, the turret turns one time.
    
    public static final int k_kickInRPS = 50;
    public static final int k_kickOutRPS = 50;

    public static final int k_secondaryKickerRPS = 60;

    public static final double k_upKickerMOI = 0.00000494 * 2;
    public static final double k_kickerMOI = 0.0000356;
    public static final double k_shooterMOI = 0.0004812;

    public static final TalonFXConfiguration k_hoodConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_shooterConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_turretConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_kickerConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_secondaryKickerConfig = new TalonFXConfiguration();

    public static final double k_hoodG = 0.1;
    public static final double k_hoodS = 0;
    public static final double k_hoodP = 0;
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

    public static final double k_turretS = 10;
    public static final double k_turretP = 0.0;
    public static final double k_turretD = 0;
    
    public static final Translation3d k_turretOffset = new Translation3d(-5.965, 7.5429, 10.3851);

    public static final DoubleNumber k_turretBounds = new DoubleNumber(-500, 500);
    public static final double[][] k_scoreHoodAngle = {
        {0, 0}
    };

    public static final double[][] k_scoreShotSpeeds = {
        {0, 0}
    };

    public static final double[][] k_scoreShotTimes = {
        {0, 0}
    };
    public static final double[][] k_passHoodAngle = {
        {0, 0}
    };

    public static final double[][] k_passShotSpeeds = {
        {0, 0}
    };

    public static final double[][] k_passShotTimes = {
        {0, 0}
    };

    public static final double k_staticShotSpeed = 40;
    public static final double k_staticShotAngle = 0;
    public static final double k_hoodMOI = 0.02635296; 
    public static final double k_hoodArmLength = 0.20860766;
    public static final double k_hoodMaxAngle = 90;

    static {
        k_shooterConfigs.Slot0.kP = k_shooterP;
        k_shooterConfigs.Slot0.kS = k_shooterS;
        k_shooterConfigs.Slot0.kV = k_shooterV;

        k_kickerConfig.Slot0.kP = k_kickerP;
        k_kickerConfig.Slot0.kS = k_kickerS;
        k_kickerConfig.Slot0.kV = k_kickerV;

        k_secondaryKickerConfig.withMotorOutput(k_secondaryKickerConfig.MotorOutput.clone().withInverted(InvertedValue.Clockwise_Positive));
        k_secondaryKickerConfig.Slot0.kP = k_secondaryKickerP;
        k_secondaryKickerConfig.Slot0.kS = k_secondaryKickerS;
        k_secondaryKickerConfig.Slot0.kV = k_secondaryKickerV;

        k_turretConfig.Slot0.kS = k_turretS;
        k_turretConfig.Slot0.kP = k_turretP;
        k_turretConfig.Slot0.kD = k_turretD;

        k_hoodConfig.Slot0.kG = k_hoodG;
        // k_hoodConfig.Slot0.kS = k_hoodS;
        // k_hoodConfig.Slot0.kP = k_hoodP;
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
    
    public static final Rectangle2d k_blueTrenchLeft = new Rectangle2d(
        new Translation2d(2.5, 8),
        new Translation2d(6.6, 6.8)
    );

    public static final Rectangle2d k_blueTrenchRight = new Rectangle2d(
        new Translation2d(2.5, 1.3),
        new Translation2d(6.6, 0)
    );

    public static final Rectangle2d k_redTrenchLeft = new Rectangle2d(
        new Translation2d(14, 8),
        new Translation2d(10.2, 6.8)
    );

    public static final Rectangle2d k_redTrenchRight = new Rectangle2d(
        new Translation2d(14, 1.3),
        new Translation2d(10.2, 0)
    );
  }

  

  public static class IntakeConstants {
    public static final int k_inRPS = 10;
    public static final int k_outRPS = -60;

    public static final int k_outPosition = 90;

    public static final TalonFXConfiguration k_intakeRollerConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration k_intakeExtensionConfigs = new TalonFXConfiguration();

    public static final double k_rollerP = 0.0;
    public static final double k_rollerS = 0.0;
    public static final double k_rollerV = 0.0;

    public static final double k_intakeExtensionG = 0;
    public static final double k_intakeExtensionP = 0;
    public static final double k_intakeExtensionV = 0;
    public static final double k_intakeExtensionD = 0;
    
    
    public static final double k_intakeRollerMOI = 0.093084;
    static {

        
        k_intakeRollerConfigs.Slot0.kP = k_rollerP;
        k_intakeRollerConfigs.Slot0.kV = k_rollerV;
        k_intakeRollerConfigs.Slot0.kS = k_rollerS;

        k_intakeExtensionConfigs.Slot0.kG = k_intakeExtensionG;
        k_intakeExtensionConfigs.Slot0.kP = k_intakeExtensionP;
        k_intakeExtensionConfigs.Slot0.kV = k_intakeExtensionV;
        k_intakeExtensionConfigs.Slot0.kD = k_intakeExtensionD;
    }

    
    }
    public static class IndexerConstants {
        public static final int k_rollerInRPS = 80;
        public static final int k_rollerOutRPS = -45;

        public static final int k_conveyorInRPS = 20; // 50
        public static final int k_conveyorOutRPS = -45;

        public static final int k_jamThreshold = 5; // should be tuned to not happen too much

        public static final TalonFXConfiguration k_indexerRollerConfigs = new TalonFXConfiguration();
        public static final TalonFXConfiguration k_indexerConveyorConfigs = new TalonFXConfiguration();
        public static final double k_indexerRollerMOI = 0.00045822;
        public static final double k_indexerConveyorMOI = 0.00001722;

        public static final double k_rollerP = 0.0;
        public static final double k_rollerS = 0.015;
        public static final double k_rollerV = 0.0367;

        public static final double k_conveyorP = 0.0;
        public static final double k_conveyorS = 0.1;
        public static final double k_conveyorV = 0.0;

        static {
            k_indexerRollerConfigs.Slot0.kP = k_rollerP;
            k_indexerRollerConfigs.Slot0.kS = k_rollerS;
            k_indexerRollerConfigs.Slot0.kV = k_rollerV;
            
            

            k_indexerConveyorConfigs.Slot0.kP = k_conveyorP;
            k_indexerConveyorConfigs.Slot0.kS = k_conveyorS;
            k_indexerConveyorConfigs.Slot0.kV = k_conveyorV;
            
        }
  }

 
    public static class Controls {
        public static final Trigger isReadyToShoot = new Trigger(Subsystems.m_hoodSubsystem.m_isOnTarget.and(Subsystems.m_shooterSubsystem.m_isAtSpeed.and(Subsystems.m_turretAimSubsystem.m_isOnTarget)));
        public static final Command extendIntake() {
            return Commands.sequence(
                Subsystems.m_intakeExtension.extendIntake(),
                Subsystems.m_intakeRoller.run(true));
        }

        public static Command shoot(){
            return Commands.either(launch(), aim(), Controls.isReadyToShoot);
        }

        public static Command aim(){
            return Commands.parallel(
                Subsystems.m_hoodSubsystem.pitchHood(() -> Subsystems.m_swerveSubsystem.distanceToTarget()),
                Subsystems.m_shooterSubsystem.shoot(() -> Subsystems.m_swerveSubsystem.distanceToTarget()),
                Subsystems.m_turretAimSubsystem.rotateToTarget()
            );

        }

        public static Command launch() {
            return Commands.parallel(
                aim(),
                Subsystems.m_indexerConveyor.run(true),
                Subsystems.m_indexerRollerSubsystem.run(true),
                Subsystems.m_secondaryKicker.run(),
                Subsystems.m_kickerSubsystem.run(true)
                );
        }

        public static Command staticLaunch() {
            return Commands.parallel(
                staticAim(),
                Subsystems.m_indexerConveyor.run(true),
                Subsystems.m_indexerRollerSubsystem.run(true),
                Subsystems.m_secondaryKicker.run(),
                Subsystems.m_kickerSubsystem.run(true)
                );
        }
        public static Command staticAim(){
            return Commands.parallel(
                Subsystems.m_hoodSubsystem.staticShoot(),
                Subsystems.m_shooterSubsystem.staticShoot(),
                Subsystems.m_turretAimSubsystem.staticAim()
            );

        }

        public static Command towerSweep() {
            return Commands.either(Commands.either(
                Commands.either(new DrivePath("OutpostDepotTowerSweep.json"), new DrivePath("OutpostDepotTowerSweepIntermediate.json"), () -> Subsystems.m_swerveSubsystem.getPose().getY() > FieldConstants.k_depotIntermediatePoint),
                Commands.either(new DrivePath("DepotOutpostTowerSweep.json"), new DrivePath("DepotOutpostTowerSweepIntermediate.json"), () -> Subsystems.m_swerveSubsystem.getPose().getY() < FieldConstants.k_outpostIntermediatePoint),
                () -> Subsystems.m_swerveSubsystem.getPose().getY() > FieldConstants.k_yCenterline),
            Commands.none(), () -> !Subsystems.m_swerveSubsystem.inTeamAlliance());    
        }
    }
}
