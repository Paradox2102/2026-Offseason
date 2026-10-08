// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveRequest;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.CommandJoystick;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

import frc.robot.Constants.Controls;
import frc.robot.Constants.Subsystems;
import frc.robot.PathPlanningThing.SelectAuto;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utils.Autos;
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
    
    public static TelemetryTable m_telemetry = Telemetry.getTable();

    public RobotContainer() {

        configureDefaults();
        configureBindings();
        // Subsystems.m_hoodSubsystem.setDefaultCommand(Subsystems.m_hoodSubsystem.staticShoot());
        // configureAutos();
        // m_controller.southFace().whileTrue(Subsystems.m_hoodSubsystem.staticShoot()).whileFalse(Subsystems.m_hoodSubsystem.returnHood());
        // m_controller.faceDown().onTrue(Subsystems.m_turretAimSubsystem.staticAim());
    }

    private void configureBindings() {
        // m_controller.faceDown().whileTrue(Commands.parallel(
        //     Subsystems.m_shooterSubsystem.staticShoot(),
        //     Commands.repeatingSequence(Commands.either(Commands.parallel(
        //         Subsystems.m_indexerConveyor.testRun(),
        //         Subsystems.m_indexerRollerSubsystem.testRun(),
        //         Subsystems.m_kickerSubsystem.testRun(),
        //         Subsystems.m_secondaryKicker.testRun()
        //     ), Commands.none(), () -> Subsystems.m_shooterSubsystem.m_isAtSpeed()))
        // ));

        m_controller.faceDown().whileTrue(Subsystems.m_intakeRoller.testRun());

        // m_controller.faceLeft().whileTrue(Subsystems.m_intakeExtension.forceRetractIntake());
        // m_controller.faceDown().onTrue(Subsystems.m_intakeExtension.extendIntake()).onFalse(Subsystems.m_intakeExtension.retractIntake());
        // m_controller.leftTrigger().onTrue(Controls.extendIntake()).onFalse(Subsystems.m_intakeRoller.stop());
        // m_controller.rightTrigger().whileTrue(Controls.shoot());
        // m_controller.leftBumper().onTrue(Controls.towerSweep());
        // m_controller.rightBumper().whileTrue(Controls.staticLaunch());

        //m_controller.southFace().whileTrue()
        
        // Subsystems.m_swerveSubsystem.registerTelemetry(logger::telemeterize);
    }

    private void configureDefaults() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        Subsystems.m_swerveSubsystem.setDefaultCommand(
            // m_swerveSubsystem will execute this command periodically
            Subsystems.m_swerveSubsystem.applyRequest(() ->
                Subsystems.m_swerveSubsystem.getDrive().withVelocityX(m_controller.getLeftY() * SwerveSubsystem.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_controller.getLeftX() * SwerveSubsystem.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_controller.getRightX() * SwerveSubsystem.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        // SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        // Subsystems.m_swerveSubsystem.setDefaultCommand(Subsystems.m_swerveSubsystem.applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_controller.getLeftY(), -m_controller.getLeftX()))));

        // Subsystems.m_indexerConveyor.setDefaultCommand(Subsystems.m_indexerConveyor.run(true));
        // Subsystems.m_indexerRollerSubsystem.setDefaultCommand(Subsystems.m_indexerRollerSubsystem.run(true));
        // Subsystems.m_kickerSubsystem.setDefaultCommand(Subsystems.m_kickerSubsystem.run(false));
        // Subsystems.m_hoodSubsystem.setDefaultCommand(Subsystems.m_hoodSubsystem.returnHood());
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //     Subsystems.m_swerveSubsystem.applyRequest(() -> idle).ignoringDisable(true)
        // );
    }
    private void configureAutos(){
        SelectAuto.addCommand("RightDoubleSweep", Autos.doubleSwipe(true));
        SelectAuto.addCommand("LeftDoubleSweep", Autos.doubleSwipe(false));

        m_telemetry.log("Auto Selection", SelectAuto.m_autoSelection);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return SelectAuto.getSelectedAuto();
    }
}
