package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TurretMathTest {
    private static final double kMin = -250.0 / 360;
    private static final double kMax = 250.0 / 360;

    @Test
    void picksTheNearestEquivalentAngle() {
        // Wanted 0.9 rot (324 degrees) is the same direction as -0.1 rot; -0.1 is nearer to 0.
        assertEquals(-0.1, TurretMath.chooseAngle(0.9, 0.0, kMin, kMax), 1e-9);
    }

    @Test
    void staysNearTheCurrentSideWhenBothFit() {
        // From +0.6 rot, the direction 0.45 rot (162 deg) is nearer at +0.45 than at -0.55.
        assertEquals(0.45, TurretMath.chooseAngle(-0.55, 0.6, kMin, kMax), 1e-9);
    }

    @Test
    void goesTheLongWayRoundAtALimit() {
        // At +0.65 rot (234 deg), the direction +0.75 rot (270 deg) is past the +250 deg limit,
        // so the turret must go round to -0.25 rot.
        assertEquals(-0.25, TurretMath.chooseAngle(0.75, 0.65, kMin, kMax), 1e-9);
    }

    @Test
    void unwindsAtTheWiringLimit() {
        // The real limits: wiring one full turn each way from the jig, soft limits 10 degrees
        // inside it (+-350), aim range 5 degrees inside those (+-345).
        double min = ShooterConstants.kTurretAimMinRot;
        double max = ShooterConstants.kTurretAimMaxRot;
        assertEquals(345.0 / 360, max, 1e-9);
        // At +340 degrees, a target that moved on to 370 is past the limit: the turret unwinds
        // back round to +10 (the same direction), not on past +345.
        assertEquals(10.0 / 360, TurretMath.chooseAngle(10.0 / 360, 340.0 / 360, min, max), 1e-9);
        // The same at the other end.
        assertEquals(-10.0 / 360, TurretMath.chooseAngle(-10.0 / 360, -340.0 / 360, min, max), 1e-9);
        // Before the limit it keeps going the short way: at +330, the direction 340 stays +340.
        assertEquals(340.0 / 360, TurretMath.chooseAngle(-20.0 / 360, 330.0 / 360, min, max), 1e-9);
        // In the middle of the range it takes the nearer way, never past a limit.
        double a = TurretMath.chooseAngle(0.9, 0.0, min, max);
        assertEquals(-0.1, a, 1e-9);
    }

    @Test
    void neverLeavesTheAimRange() {
        double min = ShooterConstants.kTurretAimMinRot;
        double max = ShooterConstants.kTurretAimMaxRot;
        for (double wanted = -3; wanted <= 3; wanted += 0.013) {
            for (double current = min; current <= max; current += 0.05) {
                double a = TurretMath.chooseAngle(wanted, current, min, max);
                assertTrue(a >= min - 1e-9 && a <= max + 1e-9, "angle " + a + " outside the range");
            }
        }
    }

    @Test
    void unreachableDirectionGivesTheNearestLimit() {
        // Limits +-90 deg: the direction 180 deg cannot be reached; stop at the nearer limit.
        assertEquals(0.25, TurretMath.chooseAngle(0.5, 0.2, -0.25, 0.25), 1e-9);
        assertEquals(-0.25, TurretMath.chooseAngle(0.5, -0.2, -0.25, 0.25), 1e-9);
    }
}
