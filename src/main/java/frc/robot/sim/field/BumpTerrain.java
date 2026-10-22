package frc.robot.sim.field;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

/**
 * Simulator only: the height of the floor, which is flat except for the four bumps.
 *
 * <p>Measured from AdvantageScope's 2026 field model (so the sim matches the picture): each bump is
 * a triangle in x, 0.165 m (6.5 in) high at its center line, with 15 degree ramps on both sides,
 * and runs in y from the field wall side to the hub. The two bumps of an alliance sit on either
 * side of its hub. No hardware: unit tested.
 */
public final class BumpTerrain {
    private BumpTerrain() {}

    public static final double kHeight = 0.165;
    /** Center line to the foot of a ramp, meters: 0.165 m / tan(15 degrees). */
    public static final double kHalfDepth = 0.625;
    /** Center lines in x: blue, red. */
    private static final double[] kCentersX = {4.6255, 11.9155};
    /** Each alliance has a bump on each side of its hub, in y. */
    private static final double[][] kRangesY = {{1.283, 3.440}, {4.629, 6.786}};
    /**
     * On a ramp the wheels cover more ground along the slope than across the floor, and they slip
     * a little as they climb: the robot moves cos(slope) x this fraction of its wheel travel.
     */
    public static final double kRampGrip = 0.9;

    /** Wheel contact points in the robot frame (the module positions, TunerConstants). */
    private static final Translation2d[] kWheels = {
        new Translation2d(Units.inchesToMeters(10.25), Units.inchesToMeters(11.75)),
        new Translation2d(Units.inchesToMeters(10.25), Units.inchesToMeters(-11.75)),
        new Translation2d(Units.inchesToMeters(-10.25), Units.inchesToMeters(11.75)),
        new Translation2d(Units.inchesToMeters(-10.25), Units.inchesToMeters(-11.75))};

    /** Floor height at a field point, meters. */
    public static double height(double x, double y) {
        double h = 0;
        for (double cx : kCentersX) {
            double dx = Math.abs(x - cx);
            if (dx < kHalfDepth && onBumpY(y)) {
                h = Math.max(h, kHeight * (1 - dx / kHalfDepth));
            }
        }
        return h;
    }

    /** dz/dx of the floor at a field point (the bumps have no slope in y). */
    public static double slopeX(double x, double y) {
        for (double cx : kCentersX) {
            double dx = x - cx;
            if (Math.abs(dx) < kHalfDepth && onBumpY(y)) {
                return -Math.signum(dx) * kHeight / kHalfDepth;
            }
        }
        return 0;
    }

    private static boolean onBumpY(double y) {
        for (double[] r : kRangesY) {
            if (y >= r[0] && y <= r[1]) {
                return true;
            }
        }
        return false;
    }

    private static Translation2d[] wheelPoints(Pose2d pose) {
        var points = new Translation2d[kWheels.length];
        for (int i = 0; i < kWheels.length; i++) {
            points[i] = pose.getTranslation().plus(kWheels[i].rotateBy(pose.getRotation()));
        }
        return points;
    }

    /**
     * The robot on the floor: height, pitch and roll from the floor under its four wheels. Pitch
     * follows WPILib: positive pitch turns the nose down, so a robot climbing a ramp nose first has
     * a negative pitch. Positive roll lifts the left side.
     */
    public static Pose3d pose3d(Pose2d pose) {
        var w = wheelPoints(pose);
        double fl = height(w[0].getX(), w[0].getY());
        double fr = height(w[1].getX(), w[1].getY());
        double bl = height(w[2].getX(), w[2].getY());
        double br = height(w[3].getX(), w[3].getY());
        double length = 2 * kWheels[0].getX();
        double width = 2 * kWheels[0].getY();
        double pitch = -Math.atan(((fl + fr) - (bl + br)) / 2 / length);
        double roll = Math.atan(((fl + bl) - (fr + br)) / 2 / width);
        double z = (fl + fr + bl + br) / 4;
        return new Pose3d(pose.getX(), pose.getY(), z, new Rotation3d(roll, pitch, pose.getRotation().getRadians()));
    }

    /** Fraction of the wheel travel that moves the robot across the floor (1 off the bumps). */
    public static double groundFactor(Pose2d pose) {
        double sum = 0;
        for (var p : wheelPoints(pose)) {
            double s = slopeX(p.getX(), p.getY());
            sum += s == 0 ? 1 : Math.cos(Math.atan(s)) * kRampGrip;
        }
        return sum / kWheels.length;
    }
}
