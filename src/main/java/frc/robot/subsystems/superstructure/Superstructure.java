package frc.robot.subsystems.superstructure;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.telemetry.Telemetry;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.Shot;
import frc.robot.subsystems.shooter.Turret;

/**
 * Makes the shooter parts work together.
 *
 * <p>Each loop it works out one shot ({@link ShotCalculator}): the target, the turret angle, the
 * flywheel speed and the hood angle. The turret always points at the target, so a shot never
 * waits for the turret to turn. The hood stays down (under the trench) until the driver shoots.
 *
 * <p>Shoot (driver holds the right trigger): the flywheel spins to the shot's speed and the hood
 * goes to the shot's angle. The balls are fed only while all three are ready: flywheel at speed,
 * hood and turret on their angles. A ball fed too early hits a slow wheel and falls short.
 */
public class Superstructure extends SubsystemBase {
    private static final double kKeepFeedingSpeedFraction = 0.85;
    private static final double kKeepFeedingTurretDeg = 3.0;
    private static final double kKeepFeedingHoodDeg = 2.0;

    private final Drive m_drive;
    private final Turret m_turret;
    private final Hood m_hood;
    private final Flywheel m_flywheel;
    private final Feeder m_feeder;
    private final IntakeRoller m_intakeRoller;
    private final ShotCalculator m_calculator = new ShotCalculator();
    /** The turret must be on target for 0.1 s: a single loop passing through the goal is not "on target". */
    private final Debouncer m_turretSteady = new Debouncer(0.1, DebounceType.RISING);
    private boolean m_turretOnTarget = false;
    private boolean m_feeding = false;
    private Shot m_shot;

    public Superstructure(Drive drive, Turret turret, Hood hood, Flywheel flywheel, Feeder feeder, IntakeRoller intakeRoller) {
        m_drive = drive;
        m_turret = turret;
        m_hood = hood;
        m_flywheel = flywheel;
        m_feeder = feeder;
        m_intakeRoller = intakeRoller;
        m_shot = compute();
        m_turret.setDefaultCommand(m_turret.track(() -> m_shot.turretRot()));
        m_hood.setDefaultCommand(m_hood.stow());
    }

    private Shot compute() {
        return m_calculator.calculate(m_drive.getPose(), m_drive.getFieldVelocity(),
            m_drive.getRobotVelocity().omega, Drive.isRedAlliance(), m_turret.getPosition());
    }

    @Override
    public void periodic() {
        m_shot = compute();
        m_turretOnTarget = m_turretSteady.calculate(m_turret.isOnTarget(m_shot.turretAimRot()));
        Telemetry.log("Shooter/Scoring", m_shot.scoring());
        Telemetry.log("Shooter/Distance m", m_shot.distance());
        Telemetry.log("Shooter/Shot Flywheel rps", m_shot.flywheelRps());
        Telemetry.log("Shooter/Shot Hood deg", m_shot.hoodDeg());
        Telemetry.log("Shooter/Ready", isReady());
        Telemetry.log("Shooter/Turret Ready", m_turretOnTarget);
        Telemetry.log("Shooter/Feeding", m_feeding);
        Telemetry.log("Shooter/Hood Ready", m_hood.isOnTarget(m_shot.hoodDeg()));
        Telemetry.getTable("Field").log("Shot Target", new Pose2d(m_shot.target(), Rotation2d.ZERO));
    }

    /** The shot of this loop. */
    public Shot getShot() {
        return m_shot;
    }

    /** Ready to start feeding: flywheel at speed, hood on its angle, turret steady on the target. */
    public boolean isReady() {
        return m_flywheel.isReady() && m_turretOnTarget && m_hood.isOnTarget(m_shot.hoodDeg());
    }

    /**
     * Keep feeding: looser than {@link #isReady}. Each ball takes some speed from the wheel; a
     * strict check would stop the feed in the middle of every ball (and pinch it), so once the
     * feed has started it only stops if the shot is clearly off.
     */
    private boolean stillGood() {
        return m_flywheel.getSpeed() >= kKeepFeedingSpeedFraction * m_shot.flywheelRps()
            && m_turret.isOnTarget(m_shot.turretAimRot(), kKeepFeedingTurretDeg)
            && m_hood.isOnTarget(m_shot.hoodDeg(), kKeepFeedingHoodDeg);
    }

    /** Start on "ready", keep on "still good" (a latch with two limits). */
    private boolean feedGate() {
        m_feeding = m_feeding ? stillGood() : isReady();
        return m_feeding;
    }

    /** Hold to shoot: spin up, raise the hood, and feed only while the shot is ready. */
    public Command shoot() {
        return Commands.parallel(
            m_flywheel.spin(() -> m_shot.flywheelRps()),
            m_hood.aim(() -> m_shot.hoodDeg()),
            m_feeder.feedWhen(this::feedGate)).finallyDo(() -> m_feeding = false).withName("Shoot");
    }

    /** Hold to eject: the feed path and the intake rollers run out. */
    public Command eject() {
        return Commands.parallel(m_feeder.eject(), m_intakeRoller.runOut()).withName("Eject");
    }
}
