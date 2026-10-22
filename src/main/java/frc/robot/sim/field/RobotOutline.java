package frc.robot.sim.field;

/**
 * Simulator only: the size of our robot, measured from its AdvantageScope model ("2102 2026
 * Offseason", CAD of 2026-10-22). Robot frame: +x = the intake side, +y = left, z up from the
 * bottom of the wheels.
 */
public final class RobotOutline {
    private RobotOutline() {}

    /** The robot without its intake: bumpers and frame. */
    public static final double kBackX = -0.406;
    public static final double kFrontX = 0.445;
    public static final double kHalfWidth = 0.441;

    /** The intake slides out and down a 20 degree slope; its front is here when it is in. */
    public static final double kIntakeInFront = 0.629 - 0.304;
    public static final double kIntakeHalfWidth = 0.363;
    private static final double kRackCos = Math.cos(Math.toRadians(20));
    private static final double kRackSin = Math.sin(Math.toRadians(20));

    /**
     * The highest point with the intake out (the turret and the intake, 0.559 m). With the intake
     * in, the intake sits higher by its travel x sin(20 degrees): 0.67 m, too tall for the trench.
     */
    public static final double kTopWithIntakeOut = 0.559;

    /** Intake front, robot frame x, meters. */
    public static double intakeFront(double rackOut) {
        return kIntakeInFront + rackOut * kRackCos;
    }

    /** The intake reaches past the bumpers. */
    public static boolean intakeOut(double rackOut) {
        return intakeFront(rackOut) > kFrontX + 0.02;
    }

    /**
     * Top of the hood, meters, by hood angle in degrees (the hood model tilted up about its axis):
     * it reaches a trench arm (0.565 m) at about 12 degrees.
     */
    private static final double[][] kHoodTop = {{0, 0.528}, {10, 0.560}, {20, 0.589}, {30, 0.615}, {40, 0.636}};

    public static double hoodTop(double hoodDeg) {
        double deg = Math.clamp(hoodDeg, 0, 40);
        for (int i = 1; i < kHoodTop.length; i++) {
            if (deg <= kHoodTop[i][0]) {
                double f = (deg - kHoodTop[i - 1][0]) / (kHoodTop[i][0] - kHoodTop[i - 1][0]);
                return kHoodTop[i - 1][1] + f * (kHoodTop[i][1] - kHoodTop[i - 1][1]);
            }
        }
        return kHoodTop[kHoodTop.length - 1][1];
    }

    /** Highest point of the robot above the floor under it, meters. */
    public static double top(double rackOut, double rackStroke, double hoodDeg) {
        return Math.max(kTopWithIntakeOut + Math.max(0, rackStroke - rackOut) * kRackSin, hoodTop(hoodDeg));
    }
}
