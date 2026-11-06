package frc.robot.utils;

import java.util.function.BooleanSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.controller.PIDController;

import bline.commands.FollowPathV2;
import bline.following.DriveType;
import bline.path.Path;
import frc.robot.Constants.Subsystems;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class Autos {
    
        private static FollowPathV2.Builder pathBuilder = new FollowPathV2.Builder(
        DriveType.SWERVE,
        Subsystems.m_swerveSubsystem,
        Subsystems.m_swerveSubsystem::getPose,
        Subsystems.m_swerveSubsystem::resetPose,
        Subsystems.m_swerveSubsystem::getChassisVelocities,
        velocities -> Subsystems.m_swerveSubsystem.driveRobotRelative(velocities, false),
        new PIDController(16.0, 0.0, 0.0),
        new PIDController(16.0, 0.0, 0.0),
        new PIDController(16.0, 0.0, 0.0) // cross-track PID. If tuned, set it here.
    );

    private static final Path doubleAuto1 = new Path("DoubleSwipe1");
    private static final Path doubleAuto2 = new Path("DoubleSwipe2");

    private static final Path driveForwardBump = new Path("driveForwardBump");
    private static final Path driveForwardTrench = new Path("driveForwardTrench");

    private static final Path testBLine = new Path("testbline");

    // public static Command doubleSwipe(BooleanSupplier isRight){
    //     return Commands.sequence(
    //         Commands.race(
    //             pathBuilder.build(doubleAuto1).withShouldFlip(SwerveSubsystem::isRedAlliance).withPoseReset().withShouldMirror(isRight),
    //             Commands.sequence(
    //                 Subsystems.m_intakeExtension.extendIntake(),
    //                 Subsystems.m_intakeRoller.run(true) 
    //             )
    //         ),
    //         Commands.parallel(
    //             Subsystems.m_hoodSubsystem.pitchHood(),
    //             Subsystems.m_shooterSubsystem.score()
    //         ).withTimeout(6),
    //         Commands.race(
    //             pathBuilder.build(doubleAuto2).withShouldFlip(SwerveSubsystem::isRedAlliance).withPoseReset().withShouldMirror(isRight),
    //             Commands.sequence(
    //                 Subsystems.m_intakeExtension.extendIntake(),
    //                 Subsystems.m_intakeRoller.run(true) 
    //             )
    //         ),
    //         Commands.parallel(
    //             Subsystems.m_hoodSubsystem.pitchHood(),
    //             Subsystems.m_shooterSubsystem.score()
    //         ).withTimeout(6)
    //     );
    // }

    public static Command driveForwardBump(BooleanSupplier isRight){
        return Commands.sequence(
            Subsystems.m_intakeExtension.extendIntake(),
            Commands.parallel(
                Subsystems.m_intakeRoller.run(true),
                pathBuilder.build(driveForwardBump).withShouldFlip(SwerveSubsystem::isRedAlliance).withPoseReset().withShouldMirror(isRight)
            )
        );
    }

    public static Command driveForwardTrench(BooleanSupplier isRight){
        return Commands.sequence(
            Subsystems.m_intakeExtension.extendIntake(),
            Commands.parallel(
                Subsystems.m_intakeRoller.run(true),
                pathBuilder.build(driveForwardTrench).withShouldFlip(SwerveSubsystem::isRedAlliance).withPoseReset().withShouldMirror(isRight)
            )
        );
    }

    public static Command testBlinePath(){
        return Commands.sequence(
            Subsystems.m_intakeExtension.extendIntake(),
            pathBuilder.build(testBLine).withPoseReset()
        );
    }
}
