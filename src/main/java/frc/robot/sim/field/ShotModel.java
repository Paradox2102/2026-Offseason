package frc.robot.sim.field;

import java.util.List;

import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;

/**
 * Simulator only: how a ball leaves our shooter, and a stationary shot flown with the same
 * physics as {@link FuelField}. No hardware: unit tested.
 *
 * <p>Exit speed = {@link #kExitSpeedFactor} x the wheel surface speed, and launch angle =
 * {@link #kLaunchAtHoodZeroDeg} - hood angle. Both are fitted to the old shot tables (the user's
 * default of 0.7 sent every ball almost straight up): they are not measured. Replace them with a
 * measured exit speed.
 */
public final class ShotModel {
    private ShotModel() {}

    public static final double kWheelDiameter = Units.inchesToMeters(4); // CAD: G4001 Fly Wheel Print
    public static final double kExitSpeedFactor = 0.24;
    public static final double kLaunchAtHoodZeroDeg = 78;
    /** The hood axis from the turret axis: along the shot 0.0837 m, up 0.4552 m (CAD). */
    public static final double kExitForward = 0.2742 - 0.1905;
    public static final double kExitHeight = 0.4552;

    /** Ball speed as it leaves, m/s. */
    public static double exitSpeed(double wheelRps) {
        return kExitSpeedFactor * Math.PI * kWheelDiameter * wheelRps;
    }

    /** Launch angle above level, radians. */
    public static double elevation(double hoodDeg) {
        return Math.toRadians(kLaunchAtHoodZeroDeg - hoodDeg);
    }

    /**
     * The direction the test shots come from: from the blue hub toward the alliance-wall corner at
     * low y, so a shot from up to 6 m away still starts on the carpet.
     */
    private static final double kFromAngle = Math.atan2(-FuelField.kHubY, -FuelField.kHubBlueX);

    /**
     * A shot from a still robot on flat floor, its turret axis {@code distance} meters from the
     * blue hub center, straight at it: where and when the ball comes down through the height of
     * the hub top ({@code dx} = along the shot, + = long). Null if it never does (too low, or it
     * hit the hub on the way up).
     */
    public static FuelField.Crossing stationaryShot(double distance, double wheelRps, double hoodDeg) {
        var field = new FuelField(FieldObstacles.kLength, FieldObstacles.kWidth, List.of(), 1);
        double speed = exitSpeed(wheelRps);
        double elevation = elevation(hoodDeg);
        double cos = Math.cos(kFromAngle);
        double sin = Math.sin(kFromAngle);
        double from = distance - kExitForward;
        double horizontal = speed * Math.cos(elevation);
        field.launch(new Translation3d(FuelField.kHubBlueX + from * cos, FuelField.kHubY + from * sin, kExitHeight),
            -horizontal * cos, -horizontal * sin, speed * Math.sin(elevation));
        for (double t = 0; t < 5 && field.crossings().isEmpty(); t += 0.02) {
            field.step(0.02, null);
        }
        var crossings = field.crossings();
        if (crossings.isEmpty()) {
            return null;
        }
        // In the frame of the shot: dx along it (+ = past the center), dy across it.
        var c = crossings.get(0);
        double along = -(c.dx() * cos + c.dy() * sin);
        double across = c.dx() * sin - c.dy() * cos;
        return new FuelField.Crossing(c.red(), along, across, c.scored(), c.flightTime());
    }
}
