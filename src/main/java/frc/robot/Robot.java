// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.subsystems.drive.SwerveSubsystem;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.hardware.power.PowerDistribution.ModuleType;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

public class Robot extends OpModeRobot {
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

    /* log PDH channel currents into the hoot log, next to the Talon signals */
    private final PowerDistribution m_pdh = new PowerDistribution(CANIDConstants.k_pdhPort, CANIDConstants.pdh, ModuleType.REV);

    // private final boolean kUseLimelight = false;

    public Robot() {
        m_robotContainer = new RobotContainer();

    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        SignalLogger.writeDoubleArray("PDH/ChannelCurrents", m_pdh.getAllCurrents(), "A");
        SignalLogger.writeDouble("PDH/Voltage", m_pdh.getVoltage(), "V");
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

        //ex: 0.13
        //ret: -0.15

        double retracted = -0.15;
        double extended = 0.13;
        double extension = 0;
        
        Pose3d intake = new Pose3d(
            new Translation3d(extension * ((extended-retracted)/1) + retracted,
                0,
                0.31 + -extension*((extended-retracted)/1) * Math.tan(Math.toRadians(-15.133))),
            new Rotation3d(0, Math.toRadians(15.133), 0));
        zeroedComponentPoses.set(new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d()});
        finalComponentPoses.set(new Pose3d[] {turret, hood, intake});
        //m_intakePublisher(Subsystems.m_intakeExtension.)
        


    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    
    public static void autonomousInit() {
        Subsystems.m_swerveSubsystem.resetPose(new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(SwerveSubsystem.isRedAlliance() ? 0 : 180))));
  
    }

    @Override
    public void simulationPeriodic() {}
}
