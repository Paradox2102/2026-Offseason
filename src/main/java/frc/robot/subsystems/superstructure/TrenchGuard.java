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
        for (int i = 0; i <= kSamples; i++) {
            double t = kLookahead * i / kSamples;
            if (near(pose.getX() + fieldVelocity.vx * t, pose.getY() + fieldVelocity.vy * t)) {
                return true;
            }
        }
        return false;
    }

    private static boolean near(double x, double y) {
        for (double[] arm : kArms) {
            if (y < arm[2] || y > arm[3]) {
                continue;
            }
            double dx = x < arm[0] ? arm[0] - x : x > arm[1] ? x - arm[1] : 0;
            if (dx < kReach + kMargin) {
                return true;
            }
        }
        return false;
    }
}
