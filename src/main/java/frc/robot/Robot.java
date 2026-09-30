// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import frc.robot.Constants.Subsystems;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
// import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.framework.TimedRobot;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;


    private StructPublisher<Pose2d> m_posePublisher = NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish();
    private StructArrayPublisher<Pose3d> zeroedComponentPoses = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Zeroed Poses", Pose3d.struct).publish();
    private StructArrayPublisher<Pose3d> finalComponentPoses = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Final Poses", Pose3d.struct).publish();
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    // private final boolean kUseLimelight = false;

    public Robot() {
        m_robotContainer = new RobotContainer();

    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        Pose2d robotPose = Subsystems.m_swerveSubsystem.getPose();

        m_posePublisher.set(robotPose);
        
        Rotation2d rot = new Rotation2d(Subsystems.m_turretAimSubsystem.getShooterAzimuth().getPosition().getValueAsDouble() * Constants.ShooterConstants.k_shooterGearReduction);
        //Rotation3d rot = new Rotation3d(Math.cos(Timer.getTimestamp()), Math.tan(Timer.getTimestamp()), Math.sin(Timer.getTimestamp()));
        Rotation3d hoodRot = new Rotation3d(Subsystems.m_hoodSubsystem.getAngle(), 0, rot.getRadians());
        //Rotation3d hoodRot = new Rotation3d(Math.sin(-Timer.getTimestamp()-100), Math.cos(-Timer.getTimestamp()-100), Math.tan(Timer.getTimestamp()-100));
        
        Pose3d turret = new Pose3d(
            new Translation3d(-0.16,
                0.19,
                0.38),
            new Rotation3d(rot));

        // FUN DO NOT REMOVE!!!!!!!!!!!!!!!!!!!!!
        // Pose3d turret = new Pose3d(
        //     new Translation3d(Math.sin(-Timer.getTimestamp()-100)*0.1, Math.cos(-Timer.getTimestamp()-100)*-0.1, Math.tan(Timer.getTimestamp()-100)*0.1),
        //     //new Rotation3d(rot)
        //     rot);

        Pose3d hood = new Pose3d(
            new Translation3d(-0.16,
                0.275,
                0.455),
            hoodRot);

        //15.133
        
        Pose3d intake = new Pose3d(
            new Translation3d(0,
                0,
                0.31 + -1 * Math.tan(Math.toRadians(-15.133))),
            new Rotation3d(0, Math.toRadians(15.133), 0));
        zeroedComponentPoses.set(new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d()});
        finalComponentPoses.set(new Pose3d[] {turret, hood, intake});
        //m_intakePublisher(Subsystems.m_intakeExtension.)
        

        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        // if (kUseLimelight) {
        //     var driveState = m_robotContainer.drivetrain.getState();
        //     double headingDeg = driveState.Pose.getRotation().getDegrees();
        //     double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        //     LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
        //     var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        //     if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        //         m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        //     }
        // }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void simulationPeriodic() {}
}
