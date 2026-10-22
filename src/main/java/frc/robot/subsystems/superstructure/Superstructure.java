package frc.robot.subsystems.superstructure;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import frc.robot.lib.util.AlertPublisher;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeRack;
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
 *
 * <p>Two rules from the robot's build, in every mode (teleop and auto):
 *
 * <ul>
 *   <li><b>Trench:</b> the robot fits under a trench arm only with the intake fully out and the
 *       hood on its lower stop. Near a trench ({@link TrenchGuard}) the intake goes out and stays
 *       out, the hood goes down (also in the middle of a shot), and nothing is fed.
 *   <li><b>Shooting:</b> the robot cannot shoot with the intake in. A shot puts the intake out and
 *       feeds only once both rack sides are fully out.
 * </ul>
 */
public class Superstructure extends SubsystemBase {
    private static final double kKeepFeedingSpeedFraction = 0.85;
    private static final double kKeepFeedingTurretDeg = 3.0;
    private static final double kKeepFeedingHoodDeg = 2.0;
    /** The hood counts as down (fits under a trench) below this, degrees. */
    private static final double kHoodDownDeg = 1.0;

    private final Drive m_drive;
    private final Turret m_turret;
    private final Hood m_hood;
    private final Flywheel m_flywheel;
    private final Feeder m_feeder;
    private final IntakeRoller m_intakeRoller;
    private final IntakeRack m_intakeRack;
    private final ShotCalculator m_calculator = new ShotCalculator();
    private final Alert m_trenchAlert = AlertPublisher.create("Health", "trench",
        "Near a trench with the intake not fully out or the hood up: the robot does not fit under the arm", Level.HIGH);
    private boolean m_nearTrench = false;
    private boolean m_shooting = false;
    /** The turret must be on target for 0.1 s: a single loop passing through the goal is not "on target". */
    private final Debouncer m_turretSteady = new Debouncer(0.1, DebounceType.RISING);
    private boolean m_turretOnTarget = false;
    private boolean m_feeding = false;
    private Shot m_shot;

    public Superstructure(Drive drive, Turret turret, Hood hood, Flywheel flywheel, Feeder feeder, IntakeRoller intakeRoller,
                          IntakeRack intakeRack) {
        m_drive = drive;
        m_turret = turret;
        m_hood = hood;
        m_flywheel = flywheel;
        m_feeder = feeder;
        m_intakeRoller = intakeRoller;
        m_intakeRack = intakeRack;
        m_shot = compute();
        m_intakeRack.holdOutWhile("trench", () -> m_nearTrench);
        m_intakeRack.holdOutWhile("shooting", () -> m_shooting);
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
        m_nearTrench = TrenchGuard.nearTrench(m_drive.getPose(), m_drive.getFieldVelocity());
        m_trenchAlert.set(m_nearTrench && (!m_intakeRack.isExtended() || m_hood.getAngleDeg() > kHoodDownDeg));
        m_turretOnTarget = m_turretSteady.calculate(m_turret.isOnTarget(m_shot.turretAimRot()));
        Telemetry.log("Shooter/Scoring", m_shot.scoring());
        Telemetry.log("Shooter/Distance m", m_shot.distance());
        Telemetry.log("Shooter/Shot Flywheel rps", m_shot.flywheelRps());
        Telemetry.log("Shooter/Shot Hood deg", m_shot.hoodDeg());
        Telemetry.log("Shooter/Ready", isReady());
        Telemetry.log("Shooter/Turret Ready", m_turretOnTarget);
        Telemetry.log("Shooter/Feeding", m_feeding);
        Telemetry.log("Shooter/Hood Ready", m_hood.isOnTarget(m_shot.hoodDeg()));
        Telemetry.log("Shooter/Near Trench", m_nearTrench);
        Telemetry.log("Shooter/Intake Extended", m_intakeRack.isExtended());
        Telemetry.getTable("Field").log("Shot Target", new Pose2d(m_shot.target(), Rotation2d.ZERO));
    }

    /** The shot of this loop. */
    public Shot getShot() {
        return m_shot;
    }

    /** Near a trench: intake out, hood down, no feed. */
    public boolean isNearTrench() {
        return m_nearTrench;
    }

    /** The hood angle to ask for: the shot's, or down near a trench. */
    private double hoodGoal() {
        return m_nearTrench ? 0 : m_shot.hoodDeg();
    }

    /** The shot may go: not near a trench, and the intake fully out (we cannot shoot with it in). */
    private boolean allowed() {
        return !m_nearTrench && m_intakeRack.isExtended();
    }

    /**
     * Ready to start feeding: allowed, flywheel at speed, hood on its angle, turret steady on the
     * target.
     */
    public boolean isReady() {
        return allowed() && m_flywheel.isReady() && m_turretOnTarget && m_hood.isOnTarget(m_shot.hoodDeg());
    }

    /**
     * Keep feeding: looser than {@link #isReady}. Each ball takes some speed from the wheel; a
     * strict check would stop the feed in the middle of every ball (and pinch it), so once the
     * feed has started it only stops if the shot is clearly off.
     */
    private boolean stillGood() {
        return allowed() && m_flywheel.getSpeed() >= kKeepFeedingSpeedFraction * m_shot.flywheelRps()
            && m_turret.isOnTarget(m_shot.turretAimRot(), kKeepFeedingTurretDeg)
            && m_hood.isOnTarget(m_shot.hoodDeg(), kKeepFeedingHoodDeg);
    }

    /** Start on "ready", keep on "still good" (a latch with two limits). */
    private boolean feedGate() {
        m_feeding = m_feeding ? stillGood() : isReady();
        return m_feeding;
    }

    /**
     * Hold to shoot: put the intake out, spin up, raise the hood, and feed only while the shot is
     * ready.
     */
    public Command shoot() {
        return Commands.parallel(
            m_flywheel.spin(() -> m_shot.flywheelRps()),
            m_hood.aim(this::hoodGoal),
            m_feeder.feedWhen(this::feedGate))
            .beforeStarting(() -> m_shooting = true)
            .finallyDo(() -> {
                m_feeding = false;
                m_shooting = false;
            }).withName("Shoot");
    }

    /**
     * Spin up and raise the hood for the shot, but do not feed. Used on the way to a shot spot,
     * so the shot is ready when the robot stops. A shoot() that follows takes over.
     */
    public Command prepare() {
        return Commands.parallel(
            m_flywheel.spin(() -> m_shot.flywheelRps()),
            m_hood.aim(this::hoodGoal)).withName("Prepare Shot");
    }

    /** Hold to eject: the feed path and the intake rollers run out. */
    public Command eject() {
        return Commands.parallel(m_feeder.eject(), m_intakeRoller.runOut()).withName("Eject");
    }
}
