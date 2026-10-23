package frc.robot.sim;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.system.Filesystem;
import org.wpilib.telemetry.Telemetry;

import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeRack;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.sim.field.FuelField;
import frc.robot.sim.field.ShotModel;

/**
 * Simulator only: the fuel, connected to our robot. It reads the true robot pose, the intake and
 * the shooter, steps {@link FuelField}, and publishes every ball for AdvantageScope.
 *
 * <p>AdvantageScope: add "Sim/Fuel" to the 3D field as "Fuel" game pieces, and use "RobotPose3d"
 * for the robot, so it climbs the bumps.
 */
public class FuelSim {
    private static final int kPreload = 8;
    /** Balls per second out of the hopper while the feeder feeds (an assumption). */
    private static final double kShotsPerSecond = 8;
    private final Drive m_drive;
    private final IntakeRack m_rack;
    private final IntakeRoller m_roller;
    private final Feeder m_feeder;
    private final Flywheel m_flywheel;
    private final Hood m_hood;
    private final Turret m_turret;
    private final FuelField m_field;
    private final StructArrayPublisher<Translation3d> m_publisher =
        NetworkTableInstance.getDefault().getStructArrayTopic("Sim/Fuel", Translation3d.struct).publish();
    private double m_shotTimer = 0;

    public FuelSim(Drive drive, IntakeRack rack, IntakeRoller roller, Feeder feeder, Flywheel flywheel, Hood hood, Turret turret) {
        m_drive = drive;
        m_rack = rack;
        m_roller = roller;
        m_feeder = feeder;
        m_flywheel = flywheel;
        m_hood = hood;
        m_turret = turret;
        m_field = new FuelField(FieldConstants.kFieldLength, FieldConstants.kFieldWidth, loadStart(), kPreload);
    }

    /** The staged balls of AdvantageScope's field model (deploy/sim/fuel-start.csv). */
    private static List<Translation2d> loadStart() {
        var out = new ArrayList<Translation2d>();
        Path path = Filesystem.getDeployDirectory().toPath().resolve("sim/fuel-start.csv");
        try {
            for (String line : Files.readAllLines(path)) {
                if (line.isBlank() || line.startsWith("#")) {
                    continue;
                }
                String[] xy = line.split(",");
                out.add(new Translation2d(Double.parseDouble(xy[0]), Double.parseDouble(xy[1])));
            }
        } catch (IOException | RuntimeException e) {
            System.out.println("FuelSim: could not read " + path + ": " + e + " (no staged fuel)");
        }
        return out;
    }

    /** Staged balls back on the floor, the preload in the hopper, the counts at 0. */
    public void reset() {
        m_field.reset();
        m_shotTimer = 0;
    }

    public void periodic() {
        var sim = m_drive.getSim();
        var pose = sim.truthPose();
        var velocity = m_drive.getFieldVelocity();
        double omega = m_drive.getRobotVelocity().omega;

        if (m_feeder.isFeeding()) {
            m_shotTimer += 0.020 * kShotsPerSecond;
            while (m_shotTimer >= 1) {
                m_shotTimer -= 1;
                shootOne(velocity.vx, velocity.vy, omega);
            }
        } else {
            m_shotTimer = 0;
        }

        m_field.step(0.020, new FuelField.RobotInput(pose, velocity.vx, velocity.vy, omega,
            m_rack.getPosition(), IntakeConstants.kRackStroke, m_roller.getTargetSpeed() > 0));

        m_publisher.set(m_field.positions());
        Telemetry.log("Sim/Fuel/Hopper", m_field.hopper());
        Telemetry.log("Sim/Fuel/Picked Up", m_field.pickedUp());
        Telemetry.log("Sim/Fuel/Shots", m_field.shots());
        Telemetry.log("Sim/Fuel/Scored Blue", m_field.scored(false));
        Telemetry.log("Sim/Fuel/Scored Red", m_field.scored(true));
    }

    /**
     * One ball out of the shooter: from the hood axis, along the turret, at the hood's angle. It
     * keeps the velocity of the point it leaves from: the robot's velocity plus its turn (the exit
     * is off the robot center, so a turning robot moves it sideways).
     */
    private void shootOne(double robotVx, double robotVy, double omega) {
        var robot = m_drive.getSim().truthPose3d();
        double turretYaw = m_turret.getPosition() * 2 * Math.PI;
        double shotYawOnRobot = turretYaw + Math.toRadians(ShooterConstants.kTurretZeroOnRobotDeg);
        Translation3d exit = robot.transformBy(new Transform3d(
            new Translation3d(ShooterConstants.kTurretOffset.getX() + ShotModel.kExitForward * Math.cos(shotYawOnRobot),
                ShooterConstants.kTurretOffset.getY() + ShotModel.kExitForward * Math.sin(shotYawOnRobot), ShotModel.kExitHeight),
            Rotation3d.ZERO)).getTranslation();
        double fieldYaw = robot.getRotation().getZ() + shotYawOnRobot;
        double elevation = ShotModel.elevation(m_hood.getAngleDeg());
        double speed = ShotModel.exitSpeed(m_flywheel.getSpeed());
        double rx = exit.getX() - robot.getX();
        double ry = exit.getY() - robot.getY();
        m_field.launch(exit,
            speed * Math.cos(elevation) * Math.cos(fieldYaw) + robotVx - omega * ry,
            speed * Math.cos(elevation) * Math.sin(fieldYaw) + robotVy + omega * rx,
            speed * Math.sin(elevation));
    }

    public FuelField getField() {
        return m_field;
    }
}
