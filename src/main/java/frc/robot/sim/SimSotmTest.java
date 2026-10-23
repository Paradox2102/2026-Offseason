package frc.robot.sim;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.system.Timer;

import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.sim.field.FuelField;
import frc.robot.subsystems.drive.Drive;

/**
 * Simulator only: the shoot-on-the-move test, for {@code tools/sim-sotm.sh}. Runs only when the
 * environment variable SIM_SOTM=1.
 *
 * <p>Blue, teleop. For each run the robot is put at its start, spun up and filled with 40 balls,
 * then drives at a fixed field velocity (or turns) while it holds the shoot button. Each run prints
 * the balls shot, the balls scored, and how far from the hub center the balls came down (the miss,
 * from {@link FuelField.Crossing}): "SOTM ..." lines. SIM_SOTM_COMPARE=1 runs each one again with
 * shoot on the move off (aimed as if still, and the speed gate off), to show what the lead does.
 * SIM_SOTM_ONLY=&lt;text&gt; runs only the runs whose name has that text.
 */
public class SimSotmTest {
    private static final boolean kCompare = "1".equals(System.getenv("SIM_SOTM_COMPARE"));
    private static final String kOnly = System.getenv().getOrDefault("SIM_SOTM_ONLY", "");
    private static final double kHubX = 4.62;
    private static final double kHubY = 4.03;
    /** Still at the start (put there, spun up, filled), then driving with no shot, seconds. */
    private static final double kSetup = 1.0;
    private static final double kAccelerate = 0.4;
    /** After the run: balls still in the air come down. */
    private static final double kSettle = 1.5;
    private static final double kBoot = 2.0;

    /** One scripted run: start pose, field velocity (or a path of velocities), turn rate, time. */
    private record Run(String name, Pose2d start, java.util.function.Function<Pose2d, Translation2d> velocity,
                       double omega, double seconds, boolean lead) {}

    private final RobotContainer m_container;
    private final Drive m_drive;
    private final List<Run> m_runs = new ArrayList<>();
    private final Timer m_timer = new Timer();
    private int m_index = -1;
    private int m_phase = 0; // 0 setup, 1 accelerate, 2 shoot, 3 settle
    private Command m_driveCommand = null;
    private Command m_shoot = null;
    private Command m_prepare = null;
    private int m_shotsAtStart = 0;
    private int m_scoredAtStart = 0;
    private double m_speedSum = 0;
    private int m_speedCount = 0;
    /** While shooting: how far each part is from its shot value, summed over the loops. */
    private double m_turretErrSum = 0;
    private double m_turretGoalErrSum = 0;
    private double m_hoodErrSum = 0;
    private double m_wheelErrSum = 0;
    private int m_readyLoops = 0;
    private final List<String> m_summary = new ArrayList<>();

    public SimSotmTest(RobotContainer container, Drive drive) {
        m_container = container;
        m_drive = drive;
        DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
        DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
        DriverStationSim.setEnabled(false); // enabled after kBoot: the devices are not up at once
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        buildRuns();
        m_timer.start();
    }

    private void addRun(String name, Pose2d start, java.util.function.Function<Pose2d, Translation2d> velocity,
                        double omega, double seconds) {
        if (!name.contains(kOnly)) {
            return;
        }
        m_runs.add(new Run(name, start, velocity, omega, seconds, true));
        if (kCompare) {
            m_runs.add(new Run(name + " (lead off)", start, velocity, omega, seconds, false));
        }
    }

    private static Translation2d hub() {
        return new Translation2d(kHubX, kHubY);
    }

    private void buildRuns() {
        // Still shots at three distances: the baseline.
        for (double d : new double[] {2.0, 3.0, 4.5}) {
            var start = hub().plus(new Translation2d(d, Rotation2d.fromDegrees(215)));
            addRun(String.format("still %.1f m", d), new Pose2d(start, Rotation2d.ZERO), p -> Translation2d.ZERO, 0, 1.5);
        }
        for (double v : new double[] {1, 2, 3, 4}) {
            double sv = v;
            // Sideways past the hub, 2.4 m from it, both ways (y 1.5 to 6.5).
            addRun(String.format("strafe +y %.0f m/s", v), new Pose2d(2.2, 1.5, Rotation2d.ZERO),
                p -> new Translation2d(0, sv), 0, 5.0 / v);
            addRun(String.format("strafe -y %.0f m/s", v), new Pose2d(2.2, 6.5, Rotation2d.ZERO),
                p -> new Translation2d(0, -sv), 0, 5.0 / v);
            // Straight at the hub (5.0 m to 1.8 m along the 215 degree line), and away from it.
            var dir = new Translation2d(1, Rotation2d.fromDegrees(35));
            var far = hub().plus(new Translation2d(5.0, Rotation2d.fromDegrees(215)));
            var near = hub().plus(new Translation2d(1.8, Rotation2d.fromDegrees(215)));
            addRun(String.format("toward %.0f m/s", v), new Pose2d(far, Rotation2d.ZERO), p -> dir.times(sv), 0, 3.2 / v);
            addRun(String.format("away %.0f m/s", v), new Pose2d(near, Rotation2d.ZERO), p -> dir.times(-sv), 0, 3.2 / v);
            // An arc round the hub, 3 m from it, from 150 to 230 degrees (clockwise seen from above).
            var arcStart = hub().plus(new Translation2d(3.0, Rotation2d.fromDegrees(150)));
            addRun(String.format("arc %.0f m/s", v), new Pose2d(arcStart, Rotation2d.ZERO), p -> arc(p, sv), 0,
                3.0 * Math.toRadians(80) / v);
        }
        // Turning in place 2.5 m out, and turning while strafing.
        var spinAt = hub().plus(new Translation2d(2.5, Rotation2d.fromDegrees(200)));
        addRun("spin 2 rad/s", new Pose2d(spinAt, Rotation2d.ZERO), p -> Translation2d.ZERO, 2.0, 2.0);
        addRun("spin 4 rad/s", new Pose2d(spinAt, Rotation2d.ZERO), p -> Translation2d.ZERO, 4.0, 2.0);
        addRun("strafe +y 2 m/s, spin 2 rad/s", new Pose2d(2.2, 1.5, Rotation2d.ZERO), p -> new Translation2d(0, 2), 2.0, 2.5);
    }

    /** Along the circle 3 m round the hub, its angle from the hub growing (150 to 230 degrees: y falls). */
    private static Translation2d arc(Pose2d pose, double speed) {
        var r = pose.getTranslation().minus(hub());
        double dist = r.getNorm();
        var radial = r.div(dist);
        var tangent = new Translation2d(-radial.getY(), radial.getX()); // angle from the hub grows
        return tangent.times(speed).plus(radial.times(2.0 * (3.0 - dist)));
    }

    public void periodic() {
        double t = m_timer.get();
        if (m_index < 0 && t < kBoot) {
            return;
        }
        if (m_index < 0 && !org.wpilib.driverstation.RobotState.isEnabled()) {
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
            return;
        }
        if (m_index < 0) {
            // First: intake out (a shot needs it), flywheel spun up.
            CommandScheduler.getInstance().schedule(m_container.getIntakeRack().extend());
            m_prepare = m_container.getSuperstructure().prepare();
            CommandScheduler.getInstance().schedule(m_prepare);
            next();
            return;
        }
        if (m_index >= m_runs.size()) {
            return;
        }
        var run = m_runs.get(m_index);
        var field = m_container.getFuelSim().getField();
        if (m_phase == 0 && t > kSetup) {
            m_driveCommand = DriveCommands.driveAt(m_drive, () -> run.velocity().apply(m_drive.getPose()), run.omega(), Rotation2d.ZERO);
            CommandScheduler.getInstance().schedule(m_driveCommand);
            m_phase = 1;
            m_timer.reset();
        } else if (m_phase == 1 && t > kAccelerate) {
            field.clearCrossings();
            m_shotsAtStart = field.shots();
            m_scoredAtStart = field.scored(false);
            m_shoot = m_container.getSuperstructure().shoot();
            CommandScheduler.getInstance().schedule(m_shoot);
            m_phase = 2;
            m_timer.reset();
        } else if (m_phase == 2) {
            var v = m_drive.getFieldVelocity();
            m_speedSum += Math.hypot(v.vx, v.vy);
            m_speedCount++;
            var sup = m_container.getSuperstructure();
            var shot = sup.getShot();
            double turret = m_container.getTurret().getPosition();
            m_turretErrSum += Math.abs(turret - shot.turretAimRot()) * 360;
            m_turretGoalErrSum += Math.abs(turret - shot.turretRot()) * 360;
            m_hoodErrSum += Math.abs(m_container.getHood().getAngleDeg() - shot.hoodDeg());
            m_wheelErrSum += Math.abs(m_container.getFlywheel().getSpeed() - shot.flywheelRps()) / shot.flywheelRps() * 100;
            if (sup.isReady()) {
                m_readyLoops++;
            }
            if ("1".equals(System.getenv("SIM_SOTM_TRACE")) && m_speedCount % 5 == 0) {
                System.out.printf("SOTM trace %s t %.2f: wheel %.1f need %.1f goal %.1f, hood %.1f need %.1f goal %.1f,"
                    + " turret %.1f aim %.1f goal %.1f deg, d %.2f rate %.2f, feeding %b%n", run.name(), t,
                    m_container.getFlywheel().getSpeed(), shot.flywheelRps(), shot.flywheelGoalRps(),
                    m_container.getHood().getAngleDeg(), shot.hoodDeg(), shot.hoodGoalDeg(),
                    turret * 360, shot.turretAimRot() * 360, shot.turretRot() * 360, shot.distance(), shot.distanceRate(),
                    m_container.getFeeder().isFeeding());
            }
            if (t > run.seconds() - kAccelerate) {
                CommandScheduler.getInstance().cancel(m_shoot);
                CommandScheduler.getInstance().cancel(m_driveCommand);
                m_prepare = m_container.getSuperstructure().prepare();
                CommandScheduler.getInstance().schedule(m_prepare);
                m_phase = 3;
                m_timer.reset();
            }
        } else if (m_phase == 3 && t > kSettle) {
            report(run, field);
            next();
        }
    }

    private void report(Run run, FuelField field) {
        int shots = field.shots() - m_shotsAtStart;
        int scored = field.scored(false) - m_scoredAtStart;
        var crossings = field.crossings();
        double missSum = 0;
        double worst = 0;
        // The mean miss split along the robot's travel (+ = the ball landed ahead of the robot's
        // motion: too little lead) and across it.
        var travel = run.velocity().apply(run.start());
        double along = 0;
        double across = 0;
        for (var c : crossings) {
            missSum += c.miss();
            worst = Math.max(worst, c.miss());
            if (travel.getNorm() > 0.1) {
                var u = travel.div(travel.getNorm());
                along += c.dx() * u.getX() + c.dy() * u.getY();
                across += -c.dx() * u.getY() + c.dy() * u.getX();
            }
        }
        if (!crossings.isEmpty()) {
            System.out.printf("SOTM %-32s   miss bias: %.2f m along the travel, %.2f m across%n", run.name(),
                along / crossings.size(), across / crossings.size());
        }
        String line = String.format("SOTM %-32s speed %.2f m/s: shots %2d, scored %2d (%3.0f %%), mean miss %.2f m, worst %.2f m,"
                + " no crossing %d", run.name(), m_speedCount == 0 ? 0 : m_speedSum / m_speedCount, shots, scored,
            shots == 0 ? 0 : 100.0 * scored / shots, crossings.isEmpty() ? Double.NaN : missSum / crossings.size(), worst,
            shots - crossings.size());
        System.out.println(line);
        int n = Math.max(1, m_speedCount);
        System.out.printf("SOTM %-32s   mean errors: turret %.1f deg off the aim (%.1f off its goal), hood %.1f deg,"
            + " flywheel %.1f %%; ready %.0f %% of the time%n", run.name(), m_turretErrSum / n, m_turretGoalErrSum / n,
            m_hoodErrSum / n, m_wheelErrSum / n, 100.0 * m_readyLoops / n);
        System.out.printf("SOTM %-32s   ended at %s%n", run.name(), m_drive.getPose());
        m_summary.add(line);
    }

    private void next() {
        m_index++;
        m_phase = 0;
        m_speedSum = 0;
        m_speedCount = 0;
        m_turretErrSum = 0;
        m_turretGoalErrSum = 0;
        m_hoodErrSum = 0;
        m_wheelErrSum = 0;
        m_readyLoops = 0;
        m_timer.reset();
        if (m_index >= m_runs.size()) {
            System.out.println("SOTM end");
            System.exit(0);
            return;
        }
        var run = m_runs.get(m_index);
        m_container.getSuperstructure().setShootOnTheMoveForTest(run.lead());
        m_drive.resetPose(run.start());
        m_container.getFuelSim().getField().fillHopper(FuelField.kHopperCapacity);
    }
}
