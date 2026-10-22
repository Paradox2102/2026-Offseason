// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.sysid.SysIdRoutine.Direction;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.Tunables;

import frc.robot.Constants.Subsystems;
import frc.robot.autos.AutoRoutines;
import frc.robot.commands.DriveCommands;
import frc.robot.health.SwerveHealth;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeRack;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.vision.Vision;

/**
 * Builds the robot: subsystems, controller bindings, dashboard buttons and autos.
 *
 * <p>Driver gamepad (port 0):
 * <ul>
 *   <li>Left stick: drive (field-centric). Right stick X: turn. The robot holds its heading while
 *       the right stick is released.
 *   <li>Right bumper (hold): face our hub while driving.
 *   <li>D-pad (hold): turn to 0, 90, 180 or 270 degrees, seen from the driver.
 *   <li>B (hold): X-lock the wheels.
 *   <li>Start: "the robot faces away from me now" (resets the heading).
 *   <li>Left trigger (hold): extend the intake and run the rollers; on release the rollers stop and
 *       the rack stays out. A: retract the intake.
 * </ul>
 *
 * <p>Test gamepad (port 2), for SysId on blocks or on carpet: bumpers pick the routine
 * (Drive/SysId Routine), A/B = quasistatic forward/reverse, X/Y = dynamic forward/reverse.
 */
public class RobotContainer {
    private final CommandGamepad m_driver = new CommandGamepad(0);
    private final CommandGamepad m_test = new CommandGamepad(2);

    private final Drive m_drive = Subsystems.m_drive;
    @SuppressWarnings("unused") // runs in its periodic()
    private final Vision m_vision = new Vision(m_drive);
    private final IntakeRack m_intakeRack = new IntakeRack();
    private final IntakeRoller m_intakeRoller = new IntakeRoller();
    private final AutoRoutines m_autos = new AutoRoutines(m_drive);

    /** Fallback for the first robot test of the closed-loop drive gains. */
    private final TunableBoolean m_openLoopTeleop = Tunables.addBoolean("Drive/Open loop teleop", false);

    private Drive.Routine m_sysIdRoutine = Drive.Routine.TRANSLATION;

    public RobotContainer() {
        // One deadband, in JoystickShaping. The Gamepad's own 0.1 deadband would come first and
        // then ours, so the stick would feel dead for longer than either value says.
        var gamepad = m_driver.getGamepad();
        gamepad.setLeftXDeadband(0);
        gamepad.setLeftYDeadband(0);
        gamepad.setRightXDeadband(0);

        configureDriver();
        configureTest();
        configureDashboard();
    }

    private void configureDriver() {
        m_drive.setDefaultCommand(DriveCommands.teleop(
            m_drive, m_driver::getLeftY, m_driver::getLeftX, m_driver::getRightX, m_openLoopTeleop));

        m_driver.rightBumper().whileTrue(DriveCommands.faceHub(m_drive, m_driver::getLeftY, m_driver::getLeftX));
        m_driver.dpadUp().whileTrue(DriveCommands.snapTo(m_drive, m_driver::getLeftY, m_driver::getLeftX, Rotation2d.fromDegrees(0)));
        m_driver.dpadLeft().whileTrue(DriveCommands.snapTo(m_drive, m_driver::getLeftY, m_driver::getLeftX, Rotation2d.fromDegrees(90)));
        m_driver.dpadDown().whileTrue(DriveCommands.snapTo(m_drive, m_driver::getLeftY, m_driver::getLeftX, Rotation2d.fromDegrees(180)));
        m_driver.dpadRight().whileTrue(DriveCommands.snapTo(m_drive, m_driver::getLeftY, m_driver::getLeftX, Rotation2d.fromDegrees(270)));
        m_driver.faceRight().whileTrue(DriveCommands.xLock(m_drive));
        m_driver.start().onTrue(DriveCommands.resetHeading(m_drive));

        // Intake: hold the trigger to extend and run the rollers in. On release the rollers stop
        // and the rack stays out. A retracts (and stops the rollers, if the trigger is held).
        m_driver.leftTrigger().whileTrue(Commands.sequence(m_intakeRack.extend(), m_intakeRoller.runIn())
            .withName("Intake"));
        m_driver.faceDown().onTrue(m_intakeRack.retract());
    }

    private void configureTest() {
        var routines = Drive.Routine.values();
        m_test.leftBumper().onTrue(Commands.runOnce(
            () -> m_sysIdRoutine = routines[(m_sysIdRoutine.ordinal() + routines.length - 1) % routines.length]).ignoringDisable(true));
        m_test.rightBumper().onTrue(Commands.runOnce(
            () -> m_sysIdRoutine = routines[(m_sysIdRoutine.ordinal() + 1) % routines.length]).ignoringDisable(true));
        m_test.faceDown().whileTrue(sysIdSelected(Direction.FORWARD, false));
        m_test.faceRight().whileTrue(sysIdSelected(Direction.REVERSE, false));
        m_test.faceLeft().whileTrue(sysIdSelected(Direction.FORWARD, true));
        m_test.faceUp().whileTrue(sysIdSelected(Direction.REVERSE, true));
    }

    /** Built when pressed, so it uses the routine picked at that moment. */
    private Command sysIdSelected(Direction direction, boolean dynamic) {
        return Commands.defer(() -> m_drive.sysId(m_sysIdRoutine, direction, dynamic), Set.of(m_drive));
    }

    private void configureDashboard() {
        for (var routine : Drive.Routine.values()) {
            String name = "Tuning/SysId " + routine.name().charAt(0) + routine.name().substring(1).toLowerCase();
            Tunables.publish(name + "/Quasistatic Forward", m_drive.sysId(routine, Direction.FORWARD, false));
            Tunables.publish(name + "/Quasistatic Reverse", m_drive.sysId(routine, Direction.REVERSE, false));
            Tunables.publish(name + "/Dynamic Forward", m_drive.sysId(routine, Direction.FORWARD, true));
            Tunables.publish(name + "/Dynamic Reverse", m_drive.sysId(routine, Direction.REVERSE, true));
        }
        Tunables.publish("Tuning/Wheel Radius", m_drive.wheelRadiusCharacterization());
        Tunables.publish("Pit/Swerve Self-Test", SwerveHealth.clockingSelfTest(m_drive));
        Tunables.publish("Pit/Wheels Forward", SwerveHealth.wheelsForward(m_drive));
        Tunables.publish("Pit/Reset Heading", DriveCommands.resetHeading(m_drive));
        if (RobotBase.isSimulation() && m_drive.getSim() != null) {
            Tunables.publish("Sim/Kick Pose", Commands.runOnce(m_drive.getSim()::kickPose).ignoringDisable(true).withName("Kick Pose"));
        }
    }

    /** Call every loop. */
    public void periodic() {
        Telemetry.log("Drive/SysId Routine", m_sysIdRoutine.name());
    }

    public IntakeRack getIntakeRack() {
        return m_intakeRack;
    }

    public IntakeRoller getIntakeRoller() {
        return m_intakeRoller;
    }

    /** For the sim smoke test: pick an auto without the dashboard. */
    public void selectAuto(String name) {
        m_autos.setDefault(name);
    }

    public Command getAutonomousCommand() {
        return m_autos.getSelected();
    }
}
