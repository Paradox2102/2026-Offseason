package frc.robot.sim.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * The shot tables against the sim's ball model: a still shot at every table distance must come
 * down near the hub center, at the time the time-of-flight table says. If ShotModel or FuelField
 * changes, solve the tables again (bisect the flywheel speed until dx = 0 at each distance).
 */
class ShotTablesTest {
    private static FuelField.Crossing shot(double d) {
        return ShotModel.stationaryShot(d, ShooterConstants.kScoreSpeeds.get(d), ShooterConstants.kScoreHood.get(d));
    }

    @Test
    void aStillShotAtEveryTableDistanceLandsOnTheCenter() {
        for (double d = 1.25; d <= 6.0; d += 0.25) {
            var c = shot(d);
            assertNotNull(c, "no crossing at " + d + " m");
            assertTrue(c.scored(), "missed at " + d + " m");
            assertEquals(0, c.dx(), 0.05, "long/short at " + d + " m");
            assertEquals(0, c.dy(), 0.01, "sideways at " + d + " m");
        }
    }

    @Test
    void betweenTheTablePointsItStillScores() {
        for (double d = 1.375; d < 6.0; d += 0.25) {
            var c = shot(d);
            assertNotNull(c, "no crossing at " + d + " m");
            assertEquals(0, c.dx(), 0.15, "long/short at " + d + " m");
        }
    }

    @Test
    void everyHoodAngleIsInsideTheHoodsTravel() {
        for (double[] p : ShooterConstants.kScoreHood.points()) {
            assertTrue(p[1] >= 0 && p[1] <= ShooterConstants.kHoodMaxDeg, "hood " + p[1] + " deg at " + p[0] + " m");
        }
    }

    @Test
    void theTimeOfFlightTableMatchesTheModel() {
        for (double d = 1.25; d <= 6.0; d += 0.25) {
            assertEquals(shot(d).flightTime(), ShooterConstants.kScoreTimeOfFlight.get(d), 0.011, "at " + d + " m");
        }
    }
}
