package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

class FeedGainsTest {
    /** Volts the feedforward asks for at a feed speed. */
    private static double volts(double kS, double kV, double rps) {
        return kS + kV * rps;
    }

    private static void assertHeadroom(String stage, double kS, double kV, double rps) {
        double v = volts(kS, kV, rps);
        assertTrue(v <= kFeedMaxFeedforwardVolts,
            String.format("%s asks %.2f V at %.1f rps: no room left for kP, so it runs as straight power", stage, v, rps));
    }

    @Test
    void everyStageLeavesVoltsForTheVelocityLoop() {
        assertHeadroom("Indexer roller (30)", kIndexerRollerKs, kIndexerRollerKv, kIndexerRollerRps);
        assertHeadroom("Top belt tunnel (31)", kBeltTunnelKs, kBeltTunnelKv, kTopBeltRps);
        assertHeadroom("Bottom belt tunnel (32)", kBeltTunnelKs, kBeltTunnelKv, kBottomBeltRps);
        assertHeadroom("Scotty rollers (45)", kScottyKs, kScottyKv, kScottyRps);
        assertHeadroom("Vertical roller (40)", kVerticalRollerKs, kVerticalRollerKv, kVerticalRollerRps);
    }

    @Test
    void everyStageHasAVelocityLoop() {
        for (double kP : new double[] {kIndexerRollerKp, kBeltTunnelKp, kScottyKp, kVerticalRollerKp}) {
            assertTrue(kP > 0, "kP " + kP);
        }
    }

    @Test
    void anUnjamReversesWellInsideItsTime() {
        // From the fastest feed speed to the eject speed, the profile must take less than half the unjam time.
        double fastest = Math.max(Math.max(kIndexerRollerRps, kTopBeltRps), Math.max(kScottyRps, kVerticalRollerRps));
        double swing = fastest * (1 - kEjectFraction);
        assertTrue(swing / kFeedAcceleration < kFeedUnjamTime / 2,
            String.format("reverse takes %.2f s of the %.2f s unjam", swing / kFeedAcceleration, kFeedUnjamTime));
    }
}
