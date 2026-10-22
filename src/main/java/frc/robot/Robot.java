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
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.net.WebServer;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Filesystem;
import org.wpilib.telemetry.Telemetry;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import frc.robot.Constants.Subsystems;
import frc.robot.constants.CanIds;
import frc.robot.health.HealthMonitor;
import frc.robot.lib.util.AlertPublisher;
import frc.robot.sim.SimSmokeTest;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;
    private SimSmokeTest m_smokeTest = null;

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
        Telemetry.log("RobotPose", Subsystems.m_drive.getPose());
        Telemetry.log("Final Poses", componentPoses(), Pose3d.struct);
        AlertPublisher.publish();
    }

    /**
     * Poses of the moving parts for the AdvantageScope 3D model (robot asset "2102 2026 Offseason"):
     * turret, hood, intake, in that order, relative to the robot. Sub-projects 2 and 3 fill in the
     * real mechanism positions as they are built (intake done). The numbers are
     * the students' poses from the old Robot.java, which match that AdvantageScope model.
     */
    private Pose3d[] componentPoses() {
        Pose3d turret = new Pose3d(-0.16, 0.19, 0.38, Rotation3d.ZERO);
        Pose3d hood = new Pose3d(-0.16, 0.275, 0.455, Rotation3d.ZERO);
        // The students' intake formula: 0 = in (x -0.15), 1 = out (x +0.13), along the model's 15 degree slide.
        double extension = m_robotContainer.getIntakeRack().getExtensionFraction();
        double travel = extension * (0.13 - (-0.15));
        Pose3d intake = new Pose3d(-0.15 + travel, 0, 0.31 - travel * Math.tan(Math.toRadians(-15.133)),
            new Rotation3d(0, Math.toRadians(15.133), 0));
        return new Pose3d[] {turret, hood, intake};
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
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
        if ("1".equals(System.getenv("SIM_SMOKE"))) {
            m_smokeTest = new SimSmokeTest(m_robotContainer, Subsystems.m_drive);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (m_smokeTest != null) {
            m_smokeTest.periodic();
        }
    }
}
