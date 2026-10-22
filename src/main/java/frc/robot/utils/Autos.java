package frc.robot.utils;

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
        pose -> {Subsystems.m_swerveSubsystem.resetPose(pose);}, // Reset Pose
        Subsystems.m_swerveSubsystem::getChassisVelocities,
        velocities -> {Subsystems.m_swerveSubsystem.driveRobotRelative(velocities, false);}, // output
        Subsystems.m_swerveSubsystem.m_xPID,
        Subsystems.m_swerveSubsystem.m_orientPID,
        new PIDController(6.0, 0.0, 0.0) // cross-track PID. If tuned, set it here.
    );

    private static Path firstStraight = new Path("first-straight");


    public static Command testBLine(){
        return Commands.sequence(
            pathBuilder.build(firstStraight).withShouldFlip(SwerveSubsystem::isRedAlliance).withPoseReset()
        );
    }

    public static Command itsNotYouItsBLineIThink(){
        return Commands.run(() -> System.out.println("Is it an auto issue?"));
    }
}
