// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.hardware.power.PowerDistribution.ModuleType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.net.WebServer;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Filesystem;
import org.wpilib.telemetry.Telemetry;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import frc.robot.Constants.Subsystems;
import frc.robot.constants.CanIds;
import frc.robot.health.HealthMonitor;
import frc.robot.lib.util.AlertPublisher;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.sim.SimSmokeTest;
import frc.robot.sim.SimSotmTest;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;
    private SimSmokeTest m_smokeTest = null;
    private SimSotmTest m_sotmTest = null;

    /*
     * AdvantageScope's robot model "2102 2026 Offseason" reads these two root topics (the names the
     * old code used): the robot pose and the poses of its moving parts (turret, hood, intake).
     */
    private final StructPublisher<Pose2d> m_posePublisher =
        NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish();
    private final StructArrayPublisher<Pose3d> m_componentPublisher =
        NetworkTableInstance.getDefault().getStructArrayTopic("Final Poses", Pose3d.struct).publish();
    /** Simulator only: the true robot pose in 3D, with the bump under its wheels. */
    private StructPublisher<Pose3d> m_pose3dPublisher = null;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    /* log PDH channel currents into the hoot log, next to the Talon signals */
    private final PowerDistribution m_pdh = new PowerDistribution(CanIds.k_pdhPort, CanIds.pdh, ModuleType.REV);

    public Robot() {
        /*
         * Logs. WPILib 2027 sends Telemetry to NetworkTables only; DataLogManager records every
         * NetworkTables value to a .wpilog file (/home/systemcore/logs, or /u/logs with a USB
         * drive), and startDataLog adds the driver station and joystick data. The CTRE devices go
         * to a separate hoot log; open both together in AdvantageScope.
         */
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        if (isReal()) {
            SignalLogger.start();
        } else {
            SignalLogger.enableAutoLogging(false); // no hoot files on the laptop in sim
        }

        // Elastic downloads its layout (deploy/elastic-layout.json) from this web server.
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        m_robotContainer = new RobotContainer();

        /* watch CAN buses, devices, battery and cameras; see health/HealthMonitor.java */
        HealthMonitor.registerDrivetrain(Subsystems.m_drive);
        HealthMonitor.start();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        SignalLogger.writeDoubleArray("PDH/ChannelCurrents", m_pdh.getAllCurrents(), "A");
        SignalLogger.writeDouble("PDH/Voltage", m_pdh.getVoltage(), "V");
        HealthMonitor.periodic();
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();

        Telemetry.log("Match Time", MatchState.getMatchTime());
        m_posePublisher.set(Subsystems.m_drive.getPose());
        m_componentPublisher.set(componentPoses());
        AlertPublisher.publish();
    }

    /**
     * Poses of the moving parts for the AdvantageScope 3D model (robot asset "2102 2026 Offseason"):
     * turret, hood, intake, in that order, relative to the robot. Sub-projects 2 and 3 fill in the
     * real mechanism positions as they are built (intake done). The numbers are
     * the students' poses from the old Robot.java, which match that AdvantageScope model.
     */
    private Pose3d[] componentPoses() {
        // Built from the CAD of 2026-10-22, where the turret points left (its jig angle), the hood
        // is down and the intake is OUT. Each pose turns or slides its part from there.

        // Turret: turns about its axis at (-152.4, +190.5) mm.
        double turretYaw = m_robotContainer.getTurret().getPosition() * 2 * Math.PI;
        Pose3d turret = new Pose3d(-0.1524, 0.1905, 0, new Rotation3d(0, 0, turretYaw));
        // Hood: rides on the turret and tilts about the flywheel axis (y 274, z 455 mm). Up = its
        // top rises (user: a raised hood hits the trench arm), which is a negative roll here.
        double hoodRad = -Math.toRadians(m_robotContainer.getHood().getAngleDeg());
        Pose3d hood = turret.transformBy(new Transform3d(new Translation3d(0, 0.2742 - 0.1905, 0.4552),
            new Rotation3d(hoodRad, 0, 0)));
        // Intake: slides along its 20 degree slope; the model is drawn fully out, so "in" is the
        // whole stroke back up the slope.
        double back = IntakeConstants.kRackStroke - m_robotContainer.getIntakeRack().getPosition();
        double slope = Math.toRadians(IntakeConstants.kRackSlopeDeg);
        Pose3d intake = new Pose3d(-back * Math.cos(slope), 0, back * Math.sin(slope), Rotation3d.ZERO);
        return new Pose3d[] {turret, hood, intake};
    }

    @Override
    public void autonomousInit() {
        if (m_robotContainer.getFuelSim() != null) {
            m_robotContainer.getFuelSim().reset(); // a match starts with the staged fuel and the preload
        }
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousExit() {
        // Commands started by path events (intake, shoot) outlive their path: stop them too.
        m_robotContainer.cancelAutoEvents();
    }

    @Override
    public void teleopInit() {
        // In a match the robot goes through disabled, which ends the auto. In practice it can go
        // straight from auto to teleop, so stop the auto here too.
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void simulationInit() {
        m_pose3dPublisher = NetworkTableInstance.getDefault().getStructTopic("RobotPose3d", Pose3d.struct).publish();
        if ("1".equals(System.getenv("SIM_SMOKE"))) {
            m_smokeTest = new SimSmokeTest(m_robotContainer, Subsystems.m_drive);
        }
        if ("1".equals(System.getenv("SIM_SOTM"))) {
            m_sotmTest = new SimSotmTest(m_robotContainer, Subsystems.m_drive);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (m_robotContainer.getFuelSim() != null) {
            m_robotContainer.getFuelSim().periodic();
            m_pose3dPublisher.set(Subsystems.m_drive.getSim().truthPose3d());
        }
        if (m_smokeTest != null) {
            m_smokeTest.periodic();
        }
        if (m_sotmTest != null) {
            m_sotmTest.periodic();
        }
    }
}
