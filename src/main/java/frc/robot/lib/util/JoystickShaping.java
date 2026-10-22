package frc.robot.lib.util;

import org.wpilib.math.util.MathUtil;

/**
 * Turns a raw stick value into a command.
 *
 * <p>Order matters: the deadband comes first, and it rescales the rest of the stick to 0..1. Then
 * the curve. If the curve came first, a squared stick at 0.3 would give 0.09, which a 0.1 deadband
 * would throw away, so the driver would lose the slow end of the stick.
 */
public final class JoystickShaping {
    private JoystickShaping() {}

    /**
     * @param raw stick value, -1 to 1
     * @param deadband stick values smaller than this give 0
     * @param exponent 1 = linear, 2 = squared, 3 = cubed (more fine control at low speed)
     * @return shaped value, -1 to 1, same sign as {@code raw}
     */
    public static double shape(double raw, double deadband, double exponent) {
        double value = MathUtil.applyDeadband(Math.clamp(raw, -1.0, 1.0), deadband);
        return Math.copySign(Math.pow(Math.abs(value), exponent), value);
    }
}
