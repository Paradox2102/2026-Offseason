package frc.robot.calibration;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.TreeMap;
import java.util.function.ToDoubleFunction;

import frc.robot.subsystems.shooter.ShotTable;

/**
 * The shot points recorded on the robot, and the tables made from them. No hardware: unit tested.
 *
 * <p>A recorded point replaces every constant point within {@link #kReplaceDistance} of it, so one
 * measured point changes the shot at once, near it, and a fully measured table replaces the
 * constants. The time of flight is optional: only points that have one change that table.
 */
public final class ShotPoints {
    private ShotPoints() {}

    /**
     * One recorded shot.
     *
     * @param distance from the turret axis to the hub center, meters
     * @param flywheelRps wheel rps
     * @param hoodDeg degrees above the lower stop
     * @param timeOfFlight seconds, NaN if not measured
     */
    public record ShotPoint(double distance, double flywheelRps, double hoodDeg, double timeOfFlight) {}

    /** A new point this close to a recorded one replaces it (the same spot shot again), meters. */
    public static final double kSameSpot = 0.1;
    /** A recorded point replaces the constant points this close to it, meters. */
    public static final double kReplaceDistance = 0.25;

    /** The points with {@code point} added (it replaces a recorded point within kSameSpot), by distance. */
    public static List<ShotPoint> add(List<ShotPoint> points, ShotPoint point) {
        var out = new ArrayList<ShotPoint>();
        for (var p : points) {
            if (Math.abs(p.distance() - point.distance()) >= kSameSpot) {
                out.add(p);
            }
        }
        out.add(point);
        out.sort(Comparator.comparingDouble(ShotPoint::distance));
        return out;
    }

    /** The points without the one nearest {@code distance} (unchanged if there are none). */
    public static List<ShotPoint> deleteNearest(List<ShotPoint> points, double distance) {
        var out = new ArrayList<>(points);
        out.stream().min(Comparator.comparingDouble(p -> Math.abs(p.distance() - distance))).ifPresent(out::remove);
        return out;
    }

    /** The recorded point nearest {@code distance}, or null. */
    public static ShotPoint nearest(List<ShotPoint> points, double distance) {
        return points.stream().min(Comparator.comparingDouble(p -> Math.abs(p.distance() - distance))).orElse(null);
    }

    /**
     * The constant table with the recorded points in it.
     *
     * @param value what each recorded point gives this table (NaN: the point does not count)
     */
    public static ShotTable merge(ShotTable constants, List<ShotPoint> points, ToDoubleFunction<ShotPoint> value) {
        var map = new TreeMap<Double, Double>();
        var recorded = points.stream().filter(p -> !Double.isNaN(value.applyAsDouble(p))).toList();
        for (double[] c : constants.points()) {
            boolean replaced = recorded.stream().anyMatch(p -> Math.abs(p.distance() - c[0]) < kReplaceDistance);
            if (!replaced) {
                map.put(c[0], c[1]);
            }
        }
        for (var p : recorded) {
            map.put(p.distance(), value.applyAsDouble(p));
        }
        return new ShotTable(map.entrySet().stream().map(e -> new double[] {e.getKey(), e.getValue()}).toArray(double[][]::new));
    }
}
