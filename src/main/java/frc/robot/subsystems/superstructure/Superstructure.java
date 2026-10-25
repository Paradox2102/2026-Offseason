package frc.robot.subsystems.superstructure;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Tunables;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import frc.robot.calibration.Calibration;
import frc.robot.calibration.ShotPoints;
import frc.robot.calibration.ShotPoints.ShotPoint;
import frc.robot.lib.util.AlertPublisher;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeRack;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.Shot;
import frc.robot.subsystems.shooter.Turret;

/**
 * Makes the shooter parts work together.
 *
 * <p>Each loop it works out one shot ({@link ShotCalculator}): the target, the turret angle, the
 * flywheel speed and the hood angle. The turret and the hood always track the shot (user,
 * 2026-10-25), so a shot never waits for them to move; near a trench the hood goes down.
 *
 * <p>Shoot (driver holds the right trigger): the flywheel spins to the shot's speed. The balls are fed only while all three are ready: flywheel at speed,
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
 *
 * <p>Pack and pulse (user, 2026-10-22): a while after the feed starts, the intake moves in a
 * little and pulses there, with its rollers in, to push the last balls to the feed
 * ({@link PackPulse}). Not near a trench. When the shot ends the intake goes fully out again.
 */
public class Superstructure extends SubsystemBase {
    private static final double kKeepFeedingSpeedFraction = 0.85;
    /** Keep feeding while the turret is within this many times its "ready" tolerance. */
    private static final double kKeepFeedingTurretFactor = 2.0;
    private static final double kKeepFeedingHoodDeg = 2.0;
    /**
     * Until the driver shoots, set the shot up for where the robot will be this far ahead, seconds:
     * on the way home the turret turns to the hub (and the hood and flywheel get its shot) before
     * the robot reaches the zone, so it can shoot as soon as it stops. While shooting, the target
     * is from where the robot is now, so a pass from the neutral zone still goes to the pass target.
     */
    private static final double kZoneLookahead = 1.0;
    /** The hood counts as down (fits under a trench) below this, degrees. */
    private static final double kHoodDownDeg = 1.0;
    /**
     * The trench alert waits this long, seconds. The hood tracks (it is up most of the time) and
     * needs ~0.3 s to come down from a far shot (22.5 deg; 35 for a pass: 0.36 s), the intake ~0.4 s
     * to go out: inside the guard's 0.6 s look ahead, so a short "not low enough" is normal.
     */
    private static final double kTrenchAlertDelay = 0.45;
    /** The intake packs this long after the first feed of a shot, seconds ... */
    private static final double kPackDelay = 1.0;
    /** ... and pulses once it has had this long to get there, seconds. */
    private static final double kPackSettleTime = 0.3;
    /**
     * While packing, the shot waits for each side at the pack position less the pulse and this,
     * meters. Sim: the rack goes up to 1.5 cm past the pulse (it cannot follow a 4 Hz sine).
     */
    private static final double kPackTolerance = 0.03;
    /**
     * No pack this much sooner than the trench guard asks for the intake: the rack needs ~0.4 s to
     * go fully out again, and must be out before the guard (and the auto's trench brake) needs it.
     * A longer look ahead for a moving robot, seconds, and more room for a still one that may
     * start fast toward the trench (the auto stops in the lane, then goes at 3.5 m/s), meters.
     */
    private static final double kPackTrenchLookahead = TrenchGuard.kLookahead + 0.6;
    private static final double kPackTrenchMargin = 0.8;

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
    private final AimGate m_aimGate = new AimGate();
    private final Alert m_noPoseAlert = AlertPublisher.create("Health", "shooter/no-pose",
        "No field position yet (no vision pose, no auto start): the turret holds still, the hood stays down, no shots",
        Level.MEDIUM);
    private final Debouncer m_trenchAlertDebouncer = new Debouncer(kTrenchAlertDelay, DebounceType.RISING);
    private boolean m_nearTrench = false;
    /** Near enough a trench that the intake must not pack (it would not be out in time). */
    private boolean m_noPackNearTrench = false;
    private boolean m_shooting = false;
    /** The turret must be on target for 0.1 s: a single loop passing through the goal is not "on target". */
    private final Debouncer m_turretSteady = new Debouncer(0.1, DebounceType.RISING);
    private boolean m_turretOnTarget = false;
    /** The flywheel must be at the shot's speed (not its goal, which leads) for 0.1 s. */
    private final Debouncer m_wheelSteady = new Debouncer(ShooterConstants.kFlywheelReadyTime, DebounceType.RISING);
    private boolean m_wheelReady = false;
    private boolean m_feeding = false;
    private Shot m_shot;
    /** Dashboard switch: off = aim as if still, and shoot only when (nearly) still. */
    private final TunableBoolean m_shootOnTheMove = Tunables.addBoolean("Shooter/Shoot On The Move", true);
    /**
     * Pack position as a fraction of each side's out stop, and the pulse. Start values (user): in
     * only a little, so the intake does not block the shots. Find the real ones on the robot.
     */
    private final TunableDouble m_packFraction = Tunables.addDouble("Intake/Pack/Fraction", 0.85);
    private final TunableDouble m_pulseAmplitude = Tunables.addDouble("Intake/Pack/Pulse Amplitude m", 0.02);
    private final TunableDouble m_pulseRate = Tunables.addDouble("Intake/Pack/Pulse Rate Hz", 4.0);
    private final PackPulse m_packPulse = new PackPulse(kPackDelay, kPackSettleTime);
    /** When the feed first started in this shot (NaN: not yet). */
    private double m_feedStart = Double.NaN;
    /** A calibration shot: the flywheel speed and hood angle to use (null: the shot's). */
    private DoubleSupplier m_flywheelOverride = null;
    private DoubleSupplier m_hoodOverride = null;
    /** Sim test only: the lead on or off whatever the dashboard says (null = the dashboard). */
    private Boolean m_leadForTest = null;

    public Superstructure(Drive drive, Turret turret, Hood hood, Flywheel flywheel, Feeder feeder, IntakeRoller intakeRoller,
                          IntakeRack intakeRack) {
        m_drive = drive;
        m_turret = turret;
        m_hood = hood;
        m_flywheel = flywheel;
        m_feeder = feeder;
        m_intakeRoller = intakeRoller;
        m_intakeRack = intakeRack;
        setShotPoints(Calibration.get().shotPoints);
        m_shot = compute();
        m_intakeRack.holdOutWhile("trench", () -> m_nearTrench);
        m_intakeRack.holdOutWhile("shooting", () -> m_shooting);
        m_intakeRack.packWhile(this::packGoal);
        m_intakeRoller.runInWhile(m_intakeRack::isPacking);
        m_turret.setDefaultCommand(m_turret.track(
            () -> m_aimGate.turretGoal(m_drive.hasFieldPose(), m_shot.turretRot(), m_turret.getPosition())));
        // Track all the time: hoodGoal() is 0 near a trench.
        m_hood.setDefaultCommand(m_hood.aim(this::hoodGoal).withName("Hood Track"));
    }

    private Shot compute() {
        m_calculator.setShootOnTheMove(m_leadForTest != null ? m_leadForTest : m_shootOnTheMove.getAsBoolean());
        return m_calculator.calculate(m_drive.getPose(), m_drive.getFieldVelocity(),
            m_drive.getRobotVelocity().omega, Drive.isRedAlliance(), m_turret.getPosition(),
            m_shooting ? 0 : kZoneLookahead);
    }

    @Override
    public void periodic() {
        m_shot = compute();
        m_nearTrench = TrenchGuard.nearTrench(m_drive.getPose(), m_drive.getFieldVelocity());
        m_noPackNearTrench = TrenchGuard.nearTrench(m_drive.getPose(), m_drive.getFieldVelocity(), kPackTrenchLookahead,
            kPackTrenchMargin);
        m_trenchAlert.set(m_trenchAlertDebouncer.calculate(
            m_nearTrench && (!m_intakeRack.isExtended() || m_hood.getAngleDeg() > kHoodDownDeg)));
        m_turretOnTarget = m_turretSteady.calculate(
            m_turret.isOnTarget(m_shot.turretAimRot(), ShotCalculator.turretToleranceDeg(m_shot.distance())));
        m_wheelReady = m_wheelSteady.calculate(m_flywheel.isNear(flywheelNeed(), ShooterConstants.kFlywheelTolerance));
        m_noPoseAlert.set(!m_drive.hasFieldPose());
        Telemetry.log("Shooter/Field Pose", m_drive.hasFieldPose());
        Telemetry.log("Shooter/Scoring", m_shot.scoring());
        Telemetry.log("Shooter/Distance m", m_shot.distance());
        Telemetry.log("Shooter/Shot Flywheel rps", m_shot.flywheelRps());
        Telemetry.log("Shooter/Shot Hood deg", m_shot.hoodDeg());
        Telemetry.log("Shooter/Ready", isReady());
        Telemetry.log("Shooter/Turret Ready", m_turretOnTarget);
        Telemetry.log("Shooter/Feeding", m_feeding);
        Telemetry.log("Shooter/Hood Ready", m_hood.isOnTarget(hoodNeed()));
        Telemetry.log("Shooter/Near Trench", m_nearTrench);
        Telemetry.log("Shooter/Intake Extended", m_intakeRack.isExtended());
        Telemetry.log("Shooter/Intake Packing", m_intakeRack.isPacking());
        Telemetry.getTable("Field").log("Shot Target", new Pose2d(m_shot.target(), Rotation2d.ZERO));
        Telemetry.getTable("Field").log("Shot Aim Point", new Pose2d(m_shot.aimPoint(), Rotation2d.ZERO));
        Telemetry.log("Shooter/Slow Enough", slowEnough());
        Telemetry.log("Shooter/Blocked By", blockedBy());
    }

    /**
     * Sim test only: the shoot-on-the-move lead on or off (the speed gate stays as the dashboard
     * says), to compare shots with and without it.
     */
    public void setShootOnTheMoveForTest(boolean lead) {
        m_leadForTest = lead;
    }

    /** The shot of this loop. */
    public Shot getShot() {
        return m_shot;
    }

    /**
     * Shot points measured on the robot ({@link Calibration}): each replaces the score table
     * points near it, from the next loop on.
     */
    public void setShotPoints(List<ShotPoint> points) {
        m_calculator.setScoreTables(
            ShotPoints.merge(ShooterConstants.kScoreSpeeds, points, ShotPoint::flywheelRps),
            ShotPoints.merge(ShooterConstants.kScoreHood, points, ShotPoint::hoodDeg),
            ShotPoints.merge(ShooterConstants.kScoreTimeOfFlight, points, ShotPoint::timeOfFlight));
    }

    /** A still score shot at this distance from the tables in use: {flywheel wheel rps, hood degrees}. */
    public double[] shotAt(double distance) {
        return m_calculator.scoreAt(distance);
    }

    /** The flywheel speed the shot needs now (for "ready"). */
    private double flywheelNeed() {
        return m_flywheelOverride != null ? m_flywheelOverride.getAsDouble() : m_shot.flywheelRps();
    }

    /** The flywheel goal (it leads the need on the move). */
    private double flywheelGoal() {
        return m_flywheelOverride != null ? m_flywheelOverride.getAsDouble() : m_shot.flywheelGoalRps();
    }

    /** The hood angle the shot needs now (for "ready"). */
    private double hoodNeed() {
        return m_hoodOverride != null ? m_hoodOverride.getAsDouble() : m_shot.hoodDeg();
    }

    /** Low enough for a trench: intake fully out and hood down. */
    public boolean isTrenchSafe() {
        return m_intakeRack.isExtended() && m_hood.getAngleDeg() <= kHoodDownDeg;
    }

    /** Near a trench: intake out, hood down, no feed. */
    public boolean isNearTrench() {
        return m_nearTrench;
    }

    /**
     * Where the intake packs now, or null: only while shooting, after the feed has run a while,
     * and not near a trench (with a longer look ahead, so the intake is out in time).
     */
    private IntakeRack.Pack packGoal() {
        if (!m_shooting || m_noPackNearTrench) {
            return null;
        }
        double offset = m_packPulse.offset(Timer.getTimestamp() - m_feedStart, pulseAmplitude(), m_pulseRate.get());
        return Double.isNaN(offset) ? null : new IntakeRack.Pack(packFraction(), offset);
    }

    private double packFraction() {
        return Math.clamp(m_packFraction.get(), 0.5, 1.0);
    }

    private double pulseAmplitude() {
        return Math.clamp(m_pulseAmplitude.get(), 0.0, 0.05);
    }

    /**
     * The intake lets a shot go: fully out, or (while it packs) each side at least at the pack
     * position less the pulse. A dead rack Talon: see {@link #allowed}.
     */
    private boolean intakeOk() {
        boolean placed = m_intakeRack.isPacking()
            ? m_intakeRack.atLeast(packFraction(), pulseAmplitude() + kPackTolerance)
            : m_intakeRack.isExtended();
        return placed || m_intakeRack.hasDeadTalon();
    }

    /** The hood angle to ask for: the shot's, or down near a trench. */
    private double hoodGoal() {
        if (m_nearTrench) {
            return 0;
        }
        if (m_hoodOverride == null && !m_drive.hasFieldPose()) {
            return AimGate.hoodGoal(false, m_shot.hoodGoalDeg());
        }
        return m_hoodOverride != null ? m_hoodOverride.getAsDouble() : m_shot.hoodGoalDeg();
    }

    /** The aim distance is inside the tables (a pass, or a calibration shot, always is). */
    private boolean inTables() {
        return !m_shot.scoring() || m_flywheelOverride != null || (m_shot.distance() >= ShooterConstants.kMinShotDistance
            && m_shot.distance() <= ShooterConstants.kMaxShotDistance);
    }

    /**
     * The shot may go: not near a trench, the intake fully out (we cannot shoot with it in), and a
     * shot at the hub only from inside our zone (the look ahead aims at it a little early), and
     * the robot not faster than the shoot-on-the-move limits.
     *
     * <p>A dead rack Talon: the intake can never count as out, so the shot does not wait for it
     * (user, 2026-10-22: we cannot collect any more, so shoot the balls we have). The shot still
     * sends the working side out.
     */
    private boolean allowed() {
        boolean inZoneForHub = !m_shot.scoring() || ShotCalculator.inOurZone(m_drive.getPose(), Drive.isRedAlliance());
        return m_drive.hasFieldPose() && !m_nearTrench && intakeOk() && inZoneForHub && slowEnough() && inTables();
    }

    /** Slow enough to shoot: under the shoot-on-the-move limits, or the still ones if that is off. */
    private boolean slowEnough() {
        var v = m_drive.getFieldVelocity();
        double speed = Math.hypot(v.vx, v.vy);
        double turn = Math.abs(m_drive.getRobotVelocity().omega);
        return m_shootOnTheMove.getAsBoolean()
            ? speed <= ShooterConstants.kMaxShootSpeed && turn <= ShooterConstants.kMaxShootTurnRate
            : speed <= ShooterConstants.kStillShootSpeed && turn <= ShooterConstants.kStillShootTurnRate;
    }

    /** What stops a shot now (the first check that fails), or "" when it is ready. For the dashboard. */
    public String blockedBy() {
        boolean inZoneForHub = !m_shot.scoring() || ShotCalculator.inOurZone(m_drive.getPose(), Drive.isRedAlliance());
        if (!m_drive.hasFieldPose()) {
            return "no field position (vision)";
        } else if (m_nearTrench) {
            return "near a trench";
        } else if (!intakeOk()) {
            return "intake not out";
        } else if (!inZoneForHub) {
            return "outside our zone";
        } else if (!slowEnough()) {
            return "moving too fast";
        } else if (!inTables()) {
            return String.format("aim distance %.1f m", m_shot.distance());
        } else if (!m_wheelReady) {
            return "flywheel";
        } else if (!m_turretOnTarget) {
            return "turret";
        } else if (!m_hood.isOnTarget(hoodNeed())) {
            return "hood";
        }
        return "";
    }

    /**
     * Ready to start feeding: allowed, flywheel at speed, hood on its angle, turret steady on the
     * target.
     */
    public boolean isReady() {
        return allowed() && m_wheelReady && m_turretOnTarget && m_hood.isOnTarget(hoodNeed());
    }

    /**
     * Keep feeding: looser than {@link #isReady}. Each ball takes some speed from the wheel; a
     * strict check would stop the feed in the middle of every ball (and pinch it), so once the
     * feed has started it only stops if the shot is clearly off.
     */
    private boolean stillGood() {
        return allowed() && m_flywheel.getSpeed() >= kKeepFeedingSpeedFraction * flywheelNeed()
            && m_turret.isOnTarget(m_shot.turretAimRot(), kKeepFeedingTurretFactor * ShotCalculator.turretToleranceDeg(m_shot.distance()))
            && m_hood.isOnTarget(hoodNeed(), kKeepFeedingHoodDeg);
    }

    /** Start on "ready", keep on "still good" (a latch with two limits). */
    private boolean feedGate() {
        m_feeding = m_feeding ? stillGood() : isReady();
        if (m_feeding && Double.isNaN(m_feedStart)) {
            m_feedStart = Timer.getTimestamp(); // the pack and pulse count from here
        }
        return m_feeding;
    }

    /**
     * Hold to shoot: put the intake out, spin up, raise the hood, and feed only while the shot is
     * ready. A while after the feed starts the intake packs and pulses. When it ends, the feed path
     * clears (runs back) and the intake goes fully out.
     */
    public Command shoot() {
        return shoot(null, null).withName("Shoot");
    }

    /**
     * A calibration shot: like {@link #shoot}, with this flywheel speed and hood angle (the turret
     * aims as usual, and the distance limits of the tables do not apply).
     */
    public Command shootWith(DoubleSupplier flywheelRps, DoubleSupplier hoodDeg) {
        return shoot(flywheelRps, hoodDeg).withName("Shoot With");
    }

    private Command shoot(DoubleSupplier flywheelRps, DoubleSupplier hoodDeg) {
        return Commands.parallel(
            m_flywheel.spin(this::flywheelGoal),
            m_hood.aim(this::hoodGoal),
            m_feeder.feedWhen(this::feedGate))
            .beforeStarting(() -> {
                m_shooting = true;
                m_flywheelOverride = flywheelRps;
                m_hoodOverride = hoodDeg;
            })
            .finallyDo(() -> {
                m_feeding = false;
                m_shooting = false;
                m_feedStart = Double.NaN;
                m_flywheelOverride = null;
                m_hoodOverride = null;
            });
    }

    /**
     * Spin up and raise the hood for the shot, but do not feed. Used on the way to a shot spot,
     * so the shot is ready when the robot stops. A shoot() that follows takes over.
     */
    public Command prepare() {
        return Commands.parallel(
            m_flywheel.spin(this::flywheelGoal),
            m_hood.aim(this::hoodGoal)).withName("Prepare Shot");
    }

    /** Hold to eject: the feed path and the intake rollers run out. */
    public Command eject() {
        return Commands.parallel(m_feeder.eject(), m_intakeRoller.runOut()).withName("Eject");
    }
}
