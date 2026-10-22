package frc.robot.subsystems.shooter;

/**
 * Picks which way round the turret turns.
 *
 * <p>A direction can be reached at many turret angles: 30 degrees, 390 degrees, -330 degrees...
 * The cable chain allows only part of that range. Of the equivalent angles inside the limits,
 * the turret takes the one nearest to where it is, so it never spins the long way round unless a
 * limit makes it.
 */
public final class TurretMath {
    private TurretMath() {}

    /**
     * @param wanted the direction to point, turret rotations (any value)
     * @param current where the turret is now, rotations
     * @param min lower limit, rotations
     * @param max upper limit, rotations
     * @return the angle to go to, inside [min, max]
     */
    public static double chooseAngle(double wanted, double current, double min, double max) {
        double best = Double.NaN;
        for (int k = -3; k <= 3; k++) {
            double candidate = wanted + k;
            if (candidate < min || candidate > max) {
                continue;
            }
            if (Double.isNaN(best) || Math.abs(candidate - current) < Math.abs(best - current)) {
                best = candidate;
            }
        }
        if (!Double.isNaN(best)) {
            return best;
        }
        // The direction cannot be reached: stop at the limit nearest to it (if both are as near,
        // the one nearer to where the turret is).
        double toMax = Math.abs(Math.IEEEremainder(wanted - max, 1.0));
        double toMin = Math.abs(Math.IEEEremainder(wanted - min, 1.0));
        if (Math.abs(toMax - toMin) < 1e-9) {
            return Math.abs(current - max) <= Math.abs(current - min) ? max : min;
        }
        return toMax < toMin ? max : min;
    }
}
