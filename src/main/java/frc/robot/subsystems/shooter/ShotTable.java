package frc.robot.subsystems.shooter;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * A value that depends on distance, from measured points (distance, value). Between two points
 * the value is interpolated in a straight line; past the ends it stays at the end value.
 */
public class ShotTable {
    private final double[][] m_points;
    private final InterpolatingDoubleTreeMap m_map = new InterpolatingDoubleTreeMap();

    /** @param points {distance m, value} pairs */
    public ShotTable(double[][] points) {
        m_points = points;
        for (double[] p : points) {
            m_map.put(p[0], p[1]);
        }
    }

    public double get(double distance) {
        return m_map.get(distance);
    }

    /** The same table with every value multiplied by {@code factor} (for a unit change). */
    public ShotTable scaled(double factor) {
        double[][] scaled = new double[m_points.length][];
        for (int i = 0; i < m_points.length; i++) {
            scaled[i] = new double[] {m_points[i][0], m_points[i][1] * factor};
        }
        return new ShotTable(scaled);
    }
}
