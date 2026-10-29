// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.CommandJoystick;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Tunables;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.PathPlanningThing.SelectAuto;
import frc.robot.Sales_because_if_6328_does_if_it_must_be_good.SalesAssociate;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.shooter.FuelLaunchSim;
import frc.robot.utils.Autos;
import frc.robot.utils.FuelSim;
public class RobotContainer {
    

    /* Setting up bindings for necessary control of the swerve drive platform */
    
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private final Telemetry logger = new Telemetry(SwerveSubsystem.MaxSpeed);

    public static final CommandGamepad m_controller = new CommandGamepad(0);
    @SuppressWarnings("unused")
    private final CommandJoystick m_joystick = new CommandJoystick(1);

    private final FuelLaunchSim launchSim = new FuelLaunchSim(Subsystems.m_swerveSubsystem::getSotmHubAimPos, Subsystems.m_swerveSubsystem::getChassisVelocities);


    public RobotContainer() {
        // configureFuelSim();
        configureDefaults();
        configureBindings();
        m_controller.faceLeft().whileTrue(Subsystems.m_hoodSubsystem.staticPitch());
        configureAutos();
    }

    private void configureBindings() {

        Trigger isReadyToShoot = new Trigger(() -> Subsystems.m_hoodSubsystem.isOnTarget() && Subsystems.m_shooterSubsystem.isAtSpeed() && m_controller.rightTrigger().getAsBoolean() && Subsystems.m_turretAimSubsystem.isOnTarget());
        Trigger m_noIntakeShoot = new Trigger(() -> isReadyToShoot.getAsBoolean() && !m_controller.leftTrigger().getAsBoolean());
        Trigger m_isConveyorAtSpeed = new Trigger(() -> m_noIntakeShoot.getAsBoolean() && Subsystems.m_indexerConveyor.getVelocity() > 20);
        isReadyToShoot.whileTrue(Commands.parallel(
            Subsystems.m_indexerConveyor.run(true),
            
            Subsystems.m_kickerSubsystem.run(),
            Subsystems.m_secondaryKicker.run(true)
            ));
            m_isConveyorAtSpeed.whileTrue(
               Subsystems.m_indexerRollerSubsystem.runPower(true)
            );
        m_noIntakeShoot.whileTrue(Commands.parallel(Subsystems.m_intakeExtension.agitate(), Subsystems.m_intakeRoller.run(true)));

        m_controller.leftTrigger().whileTrue(Commands.sequence(
            Subsystems.m_intakeExtension.extendIntake(),
            Subsystems.m_intakeRoller.run(true)
        ));
        m_controller.faceUp().whileTrue(Commands.sequence(
            Subsystems.m_intakeExtension.extendIntake(),
            Subsystems.m_intakeRoller.run(false)));
        // m_controller.faceLeft().whileTrue(
        //     Subsystems.m_shooterSubsystem.staticShoot()
        // );
        //         m_controller.faceLeft().onTrue(
        //     Subsystems.m_intakeExtension.retractIntake()
        // );

        m_controller.rightTrigger().whileTrue(Commands.parallel(
            Subsystems.m_shooterSubsystem.shoot(),
            Subsystems.m_hoodSubsystem.pitchHood()
            
        )).onFalse(Commands.parallel(
        Subsystems.m_indexerConveyor.runSlow(false),
        Subsystems.m_indexerRollerSubsystem.runSlow(false),
       Subsystems.m_kickerSubsystem.runSlow(),
        Subsystems.m_secondaryKicker.runSlow(false)
        ).withTimeout(0.5));
            
        m_controller.faceDown().whileTrue(Commands.parallel( // Shooter override
            Subsystems.m_indexerConveyor.run(true),
            Subsystems.m_indexerRollerSubsystem.run(true),
            Subsystems.m_kickerSubsystem.run(),
           
            Subsystems.m_secondaryKicker.run(true)//,
            // Subsystems.m_shooterSubsystem.staticShoot(),
            // Subsystems.m_hoodSubsystem.staticPitch()
        ));

        m_joystick.getHID().button(0).whileTrue(Commands.parallel(Subsystems.m_intakeExtension.retractIntake(), Subsystems.m_intakeRoller.run(true)));
        // m_joystick.getHID().button(7).onTrue(Subsystems.m_intakeExtension.extendIntakeAndReset());
        m_joystick.getHID().button(1).whileTrue(Commands.parallel(
                Subsystems.m_secondaryKicker.run(false),
                Subsystems.m_indexerConveyor.run(false)
        ));
        m_joystick.getHID().button(6).whileTrue(Commands.run(
            () -> {Subsystems.m_turretAimSubsystem.setShutoff(!Subsystems.m_turretAimSubsystem.getShutoff());}
        ));

        m_joystick.getHID().button(10).onTrue(Commands.run(() -> SalesAssociate.careAboutCustomerService(!SalesAssociate.getCareAboutCustomerService())));
        m_joystick.getHID().button(9).onTrue(Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
        }).ignoringDisable(true));



        // m_controller.leftBumper().onTrue(Subsystems.m_intakeExtension.extendIntake());
        // m_controller.leftTrigger().whileTrue(Subsystems.m_intakeRoller.testRun(true));

        // m_controller.rightBumper().whileTrue(Commands.parallel(
        //     Subsystems.m_shooterSubsystem.staticShoot(),
        //     Subsystems.m_indexerConveyor.testRun(),
        //         Subsystems.m_kickerSubsystem.testRun(),
        //         Subsystems.m_secondaryKicker.testRun(),
        //     Commands.repeatingSequence(Commands.either(Commands.parallel(
        //         Subsystems.m_indexerRollerSubsystem.testRun(true)
        //     ), Commands.none(), () -> Subsystems.m_shooterSubsystem.m_isAtSpeed()))
        // ));

        // m_controller.rightTrigger().whileTrue(Commands.parallel(Subsystems.m_intakeRoller.testRun(false), Subsystems.m_indexerRollerSubsystem.testRun(false)));
        // m_controller.faceLeft().whileTrue(Subsystems.m_indexerRollerSubsystem.testRun(true));
        m_joystick.getHID().button(5).onTrue(Commands.runOnce(() -> Subsystems.m_swerveSubsystem.resetPose(new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(SwerveSubsystem.isRedAlliance() ? 180 : 0))))));
        
        // m_controller.dpadUp().onTrue(Subsystems.m_intakeExtension.retractIntake());
        // m_controller.faceDown().whileTrue(Commands.parallel(
        //     Subsystems.m_shooterSubsystem.staticShoot(),
        //     Subsystems.m_hoodSubsystem.staticShoot(),
        //     Commands.repeatingSequence(
        //         Commands.either(Commands.parallel(
        //             Subsystems.m_indexerConveyor.testRun(),
        //         Subsystems.m_kickerSubsystem.testRun(),
        //         Subsystems.m_secondaryKicker.testRun()
        //         ), Commands.none(), () -> Subsystems.m_shooterSubsystem.m_isAtSpeed() && Subsystems.m_hoodSubsystem.m_isOnTarget())
        //     )
        // ));
    }

    private void configureDefaults() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        Subsystems.m_swerveSubsystem.setDefaultCommand(
            // m_swerveSubsystem will execute this command periodically
            Subsystems.m_swerveSubsystem.applyRequest(() ->
                Subsystems.m_swerveSubsystem.getDrive().withVelocityX(-(m_controller.getLeftY() * Math.abs(m_controller.getLeftY())) * SwerveSubsystem.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-(m_controller.getLeftX() * Math.abs(m_controller.getLeftX())) * SwerveSubsystem.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(Math.pow(m_controller.getRightX(), 4) * Math.signum(-m_controller.getRightX()) * SwerveSubsystem.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        Subsystems.m_turretAimSubsystem.setDefaultCommand(Subsystems.m_turretAimSubsystem.rotateToTarget());


        // Subsystems.m_indexerConveyor.setDefaultCommand(Subsystems.m_indexerConveyor.runSlow(false));
        // Subsystems.m_indexerRollerSubsystem.setDefaultCommand(Subsystems.m_indexerRollerSubsystem.runSlow(false));
        // Subsystems.m_kickerSubsystem.setDefaultCommand(Subsystems.m_kickerSubsystem.runSlow());
        // Subsystems.m_secondaryKicker.setDefaultCommand(Subsystems.m_secondaryKicker.runSlow(false));
        // Subsystems.m_hoodSubsystem.setDefaultCommand(Subsystems.m_hoodSubsystem.returnHood());
       
    }

    
    private void configureAutos(){
        // SelectAuto.addCommand("RightDoubleSweep", Autos.doubleSwipe(true));
        SelectAuto.addCommand("Double Swipe", Autos.doubleSwipe());

        Tunables.publish("Auto Selection", SelectAuto.m_autoSelection);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return SelectAuto.getSelectedAuto();
    }

    private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(0.876, 0.826, 0.4, Subsystems.m_swerveSubsystem::getSotmHubAimPos, () ->  Subsystems.m_swerveSubsystem.getChassisVelocities().toFieldRelative(Subsystems.m_swerveSubsystem.getPigeon2().getRotation2d()));

    instance.start();
    // instance.clearFuel();
//     Telemetry.put(Commands.runOnce(() -> {
//                 FuelSim.getInstance().clearFuel();
//                 FuelSim.getInstance().spawnStartingFuel();
//             })
//             .withName("Reset Fuel")
//             .ignoringDisable(true));
  }
}
