// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.PathPlanningThing.SelectAuto;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utils.Constants.Controls;
import frc.robot.utils.Constants.Subsystems;

public class RobotContainer {
    

    /* Setting up bindings for necessary control of the swerve drive platform */
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(SwerveSubsystem.MaxSpeed);

    public static final CommandXboxController m_controller = new CommandXboxController(0);
    private final CommandJoystick m_joystick = new CommandJoystick(1);

    public final SwerveSubsystem m_swerveSubsystem = Subsystems.m_swerveSubsystem;

    /* Path follower */
    private final AutoFactory autoFactory;
    // private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        autoFactory = m_swerveSubsystem.createAutoFactory();
        // autoRoutines = new AutoRoutines(autoFactory);

        // autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureDefaults();
        configureBindings();
        configureAutos();

        // Warmup Choreo to avoid Java pauses
        CommandScheduler.getInstance().schedule(autoFactory.warmupCmd());
    }

    private void configureBindings() {
        
        
        // m_controller.a().onTrue(new DrivePath("test.json"));
        // m_controller.b().whileTrue(Commands.run(() -> {new ChassisSpeeds(2, 0, 0);}, m_swerveSubsystem));
        
        m_controller.leftTrigger().onTrue(Controls.extendIntake()).onFalse(Subsystems.m_intakeRoller.stop());
        m_controller.rightTrigger().whileTrue(Controls.shoot());
        m_controller.leftBumper().onTrue(Controls.towerSweep());
        m_controller.rightBumper().whileTrue(Controls.staticLaunch());
        m_swerveSubsystem.registerTelemetry(logger::telemeterize);
    }

    private void configureDefaults() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_swerveSubsystem.setDefaultCommand(
            // m_swerveSubsystem will execute this command periodically
            m_swerveSubsystem.applyRequest(() ->
                m_swerveSubsystem.getDrive().withVelocityX(-m_controller.getLeftY() * SwerveSubsystem.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_controller.getLeftX() * SwerveSubsystem.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_controller.getRightX() * SwerveSubsystem.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        Subsystems.m_indexerConveyor.setDefaultCommand(Subsystems.m_indexerConveyor.run(true));
        Subsystems.m_indexerRollerSubsystem.setDefaultCommand(Subsystems.m_indexerRollerSubsystem.run(true));
        Subsystems.m_kickerSubsystem.setDefaultCommand(Subsystems.m_kickerSubsystem.run(false));
        Subsystems.m_hoodSubsystem.setDefaultCommand(Subsystems.m_hoodSubsystem.returnHood());
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_swerveSubsystem.applyRequest(() -> idle).ignoringDisable(true)
        );
    }
    private void configureAutos(){
        SmartDashboard.putData("Auto Selection", SelectAuto.m_autoSelection);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return SelectAuto.getSelectedAuto();
    }
}
