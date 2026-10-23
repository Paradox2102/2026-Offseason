package frc.robot.subsystems.superstructure;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import frc.robot.constants.FieldConstants;

/**
 * Is the robot in, or about to drive into, a trench lane?
 *
 * <p>The trench arms are 0.565 m (22.25 in) above the carpet. Our robot fits under them only with
 * the intake fully out (in, the intake sits 0.11 m higher) and the hood on its lower stop (the hood
 * rises as it tilts up). So near a trench the intake must be out and the hood down: the
 * superstructure does that while this says "near".
 *
 * <p>Each trench has a lane along the field wall, under its arm, and a solid wall between the lane
 * and the bump: a robot can only be under the arm from inside the lane. So "near" = the robot's
 * center is in a lane's y range and the robot reaches within a margin of the arm in x, now or
 * within {@link #kLookahead} seconds at its present velocity (time to get the intake out and the
 * hood down). Positions from the 2026 field model (AdvantageScope); red is blue turned 180 degrees
 * about the field center. No hardware: unit tested.
 */
public final class TrenchGuard {
    private TrenchGuard() {}

    /** Blue trench arms: {arm min x, arm max x, lane min y, lane max y}, meters. */
    private static final double[][] kBlueArms = {
        {4.435, 4.816, 0.0, 1.279}, // along the low-y wall (the right trench, seen from the blue wall)
        {4.549, 4.702, 6.790, 8.069}, // along the high-y wall
    };
    /** Farthest point of the robot from its center: the intake front (0.63 m) or a body corner. */
    public static final double kReach = 0.65;
    /** Extra room, meters. */
    public static final double kMargin = 0.25;
    /** Look this far ahead along the velocity, seconds: the intake takes ~0.45 s, the hood ~0.3 s. */
    public static final double kLookahead = 0.6;
    private static final int kSamples = 6;

    /** All four trench arms, both alliances, in the field frame (blue origin). */
    private static double[][] arms() {
        double length = FieldConstants.kFieldLength;
        double width = FieldConstants.kFieldWidth;
        var out = new double[kBlueArms.length * 2][];
        for (int i = 0; i < kBlueArms.length; i++) {
            double[] a = kBlueArms[i];
            out[2 * i] = a;
            out[2 * i + 1] = new double[] {length - a[1], length - a[0], width - a[3], width - a[2]};
        }
        return out;
    }

    private static final double[][] kArms = arms();

    /** The robot is in a trench lane near its arm, now or soon. */
    public static boolean nearTrench(Pose2d pose, ChassisVelocities fieldVelocity) {
        return nearTrench(pose, fieldVelocity, kLookahead, 0);
    }

    /**
     * The same, looking {@code lookahead} seconds ahead, with {@code extraMargin} meters more room
     * (for something slower than the guard allows for, or a robot that may start fast from still).
     */
    public static boolean nearTrench(Pose2d pose, ChassisVelocities fieldVelocity, double lookahead, double extraMargin) {
        for (int i = 0; i <= kSamples; i++) {
            double t = lookahead * i / kSamples;
            if (near(pose.getX() + fieldVelocity.vx * t, pose.getY() + fieldVelocity.vy * t, kMargin + extraMargin)) {
                return true;
            }
        }
        return false;
    }

    /** Robot footprint, robot frame, meters: rear bumper, front of the intake when out, half width. */
    private static final double kRobotBack = 0.406;
    private static final double kRobotFront = 0.73;
    private static final double kRobotHalfWidth = 0.441;
    /** The auto counts on stopping at this, m/s^2 (the paths use 6). */
    public static final double kBrakeDecel = 4.0;
    /** And stopping this far short of the arm, meters. */
    public static final double kBrakeMargin = 0.10;

    /**
     * In a trench lane, heading for its arm, and so close that, braking at {@link #kBrakeDecel},
     * the robot would only just stop {@link #kBrakeMargin} short of it. An auto that may go under
     * only with the intake out and the hood down stops here until they are: it can start fast and
     * slow down only if the intake is late. False once any part is under the arm (stopping there
     * does not help).
     */
    public static boolean mustBrake(Pose2d pose, ChassisVelocities fieldVelocity) {
        // The footprint's extent in field x, from the robot center.
        double cos = pose.getRotation().getCos();
        double sin = pose.getRotation().getSin();
        double minDx = Double.POSITIVE_INFINITY;
        double maxDx = Double.NEGATIVE_INFINITY;
        for (double lx : new double[] {-kRobotBack, kRobotFront}) {
            for (double ly : new double[] {-kRobotHalfWidth, kRobotHalfWidth}) {
                double dx = lx * cos - ly * sin;
                minDx = Math.min(minDx, dx);
                maxDx = Math.max(maxDx, dx);
            }
        }
        double x = pose.getX();
        double vx = fieldVelocity.vx;
        double stopping = vx * vx / (2 * kBrakeDecel) + kBrakeMargin;
        for (double[] arm : kArms) {
            if (pose.getY() < arm[2] || pose.getY() > arm[3]) {
                continue;
            }
            double gap;
            if (x < arm[0] && vx >= 0) {
                gap = arm[0] - (x + maxDx); // west of the arm, not driving away
            } else if (x > arm[1] && vx <= 0) {
                gap = (x + minDx) - arm[1]; // east of it
            } else {
                continue;
            }
            if (gap >= 0 && gap < stopping) {
                return true;
            }
        }
        return false;
    }

    private static boolean near(double x, double y, double margin) {
        for (double[] arm : kArms) {
            if (y < arm[2] || y > arm[3]) {
                continue;
            }
            double dx = x < arm[0] ? arm[0] - x : x > arm[1] ? x - arm[1] : 0;
            if (dx < kReach + margin) {
                return true;
            }
        }
        return false;
    }
}
