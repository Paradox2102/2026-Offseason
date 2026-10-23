package frc.robot.calibration;

import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

import frc.robot.calibration.ShotPoints.ShotPoint;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.superstructure.Superstructure;

/**
 * Shot table: the robot stands at a spot; jog the flywheel and the hood until the shots go in,
 * then record the point. The turret aims by itself.
 *
 * <p>Test pad: D-pad up/down flywheel (bumper held: bigger steps), left/right hood, RT shoot, A
 * record, B delete the nearest recorded point, X start again from the table at this distance, Y
 * save.
 *
 * <p>When the distance moves more than {@link #kReloadDistance} from where the values were loaded
 * (the robot moved, or a new distance was typed), the jogged values are dropped and the table's
 * values for the new distance are loaded: each spot starts from the table.
 */
public class ShotTableProcedure extends Procedure {
    private static final double kFlywheelStep = 1.0; // wheel rps
    private static final double kFlywheelBigStep = 5.0;
    private static final double kHoodStep = 0.5; // degrees
    private static final double kHoodBigStep = 2.0;
    /** A move this far from where the values were loaded loads the table again, meters. */
    static final double kReloadDistance = 0.1;

    private final Calibration m_calibration;
    private final Superstructure m_superstructure;
    private final Flywheel m_flywheel;
    private final Hood m_hood;
    private final TunableDouble m_typedDistance = Tunables.addDouble("Calibration/Shot/Typed Distance m", 0);
    private final TunableDouble m_typedTimeOfFlight = Tunables.addDouble("Calibration/Shot/Typed Time of Flight s", 0);
    private double m_flywheelRps = Double.NaN;
    private double m_hoodDeg = Double.NaN;
    /** The distance at which the values were last loaded from the table. */
    private double m_loadedAt = Double.NaN;
    private String m_last = "";

    ShotTableProcedure(Calibration calibration, Superstructure superstructure, Flywheel flywheel, Hood hood) {
        super("Shot table");
        m_calibration = calibration;
        m_superstructure = superstructure;
        m_flywheel = flywheel;
        m_hood = hood;
        m_instructions = "Stand at a spot (vision on, or type the distance). D-pad up/down: flywheel; left/right: hood"
            + " (bumper: bigger steps). RT: shoot. A: record. B: delete the nearest point. X: back to the table. Y: save."
            + " A move of 0.1 m loads the table again";
    }

    @Override
    void bind(CommandGamepad pad, Trigger active) {
        pad.dpadUp().and(active).onTrue(Commands.runOnce(() -> jogFlywheel(pad.rightBumper().getAsBoolean() ? kFlywheelBigStep : kFlywheelStep)));
        pad.dpadDown().and(active).onTrue(Commands.runOnce(() -> jogFlywheel(pad.rightBumper().getAsBoolean() ? -kFlywheelBigStep : -kFlywheelStep)));
        pad.dpadRight().and(active).onTrue(Commands.runOnce(() -> jogHood(pad.rightBumper().getAsBoolean() ? kHoodBigStep : kHoodStep)));
        pad.dpadLeft().and(active).onTrue(Commands.runOnce(() -> jogHood(pad.rightBumper().getAsBoolean() ? -kHoodBigStep : -kHoodStep)));
        pad.rightTrigger().and(active).whileTrue(shoot());
        pad.faceDown().and(active).onTrue(Commands.runOnce(this::record));
        pad.faceRight().and(active).onTrue(Commands.runOnce(this::deleteNearest));
        pad.faceLeft().and(active).onTrue(Commands.runOnce(this::fromTable));
    }

    /** The shot with the jogged flywheel speed and hood angle (the turret aims by itself). */
    public org.wpilib.command2.Command shoot() {
        return m_superstructure.shootWith(this::flywheelRps, this::hoodDeg).withName("Calibration Shot");
    }

    /** Distance to the hub: typed, or from the pose. */
    public double distance() {
        double typed = m_typedDistance.get();
        return typed > 0 ? typed : m_superstructure.getShot().distance();
    }

    public double flywheelRps() {
        followDistance();
        return m_flywheelRps;
    }

    public double hoodDeg() {
        followDistance();
        return m_hoodDeg;
    }

    /** Sim test: the typed distance, meters (0 = the pose). */
    public void setTypedDistance(double meters) {
        m_typedDistance.set(meters);
    }

    /** Load the table's values the first time, and again once the distance has moved. */
    private void followDistance() {
        double distance = distance();
        if (Double.isNaN(m_flywheelRps) || Double.isNaN(m_hoodDeg) || Double.isNaN(m_loadedAt)) {
            fromTable();
        } else if (movedFrom(m_loadedAt, distance)) {
            fromTable();
            m_last = String.format("moved to %.2f m: table values loaded", distance);
        }
    }

    /** True when the distance now is more than {@link #kReloadDistance} from {@code loadedAt}. */
    static boolean movedFrom(double loadedAt, double distance) {
        return Math.abs(distance - loadedAt) > kReloadDistance;
    }

    public void jogFlywheel(double rps) {
        m_flywheelRps = Math.max(0, flywheelRps() + rps);
    }

    public void jogHood(double deg) {
        m_hoodDeg = Math.clamp(hoodDeg() + deg, 0, ShooterConstants.kHoodMaxDeg);
    }

    /** Start again from the table's values at this distance. */
    public void fromTable() {
        m_loadedAt = distance();
        var shot = m_superstructure.shotAt(m_loadedAt);
        m_flywheelRps = shot[0];
        m_hoodDeg = shot[1];
    }

    /** Record the jogged values at this distance; the robot uses them at once. */
    public void record() {
        double tof = m_typedTimeOfFlight.get();
        var point = new ShotPoint(distance(), flywheelRps(), hoodDeg(), tof > 0 ? tof : Double.NaN);
        var data = m_calibration.data();
        data.shotPoints = ShotPoints.add(data.shotPoints, point);
        m_superstructure.setShotPoints(data.shotPoints);
        m_last = String.format("recorded %.2f m: %.1f rps, %.1f deg%s", point.distance(), point.flywheelRps(), point.hoodDeg(),
            Double.isNaN(point.timeOfFlight()) ? "" : String.format(", %.2f s", point.timeOfFlight()));
    }

    public void deleteNearest() {
        var data = m_calibration.data();
        var nearest = ShotPoints.nearest(data.shotPoints, distance());
        if (nearest == null) {
            m_last = "no recorded point to delete";
            return;
        }
        data.shotPoints = ShotPoints.deleteNearest(data.shotPoints, distance());
        m_superstructure.setShotPoints(data.shotPoints);
        m_last = String.format("deleted the point at %.2f m", nearest.distance());
    }

    @Override
    void periodic() {
        var points = m_calibration.data().shotPoints;
        m_step = m_typedDistance.get() > 0 ? "Typed distance" : "Distance from the pose";
        m_readings[0] = String.format("Distance %.2f m, %d points recorded", distance(), points.size());
        m_readings[1] = String.format("Flywheel %.1f rps (now %.1f)", flywheelRps(), m_flywheel.getSpeed());
        m_readings[2] = String.format("Hood %.1f deg (now %.1f)", hoodDeg(), m_hood.getAngleDeg());
        m_readings[3] = m_last;
    }
}
