package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

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
    void unreachableDirectionGivesTheNearestLimit() {
        // Limits +-90 deg: the direction 180 deg cannot be reached; stop at the nearer limit.
        assertEquals(0.25, TurretMath.chooseAngle(0.5, 0.2, -0.25, 0.25), 1e-9);
        assertEquals(-0.25, TurretMath.chooseAngle(0.5, -0.2, -0.25, 0.25), 1e-9);
    }
}
