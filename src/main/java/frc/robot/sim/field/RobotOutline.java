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

    /** Highest point of the robot above the floor under it, meters. */
    public static double top(double rackOut, double rackStroke) {
        return kTopWithIntakeOut + Math.max(0, rackStroke - rackOut) * kRackSin;
    }
}
