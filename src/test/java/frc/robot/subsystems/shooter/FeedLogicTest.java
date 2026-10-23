package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.shooter.FeedLogic.Mode;
import frc.robot.subsystems.shooter.FeedLogic.Out;

class FeedLogicTest {
    private static final Out F = Out.FEED;
    private static final Out R = Out.REVERSE;
    private static final Out B = Out.BRAKE;
    private static final boolean[] kFine = {false, false, false};

    // 3 stages; the last one feeds the flywheel. Run-on 0.25 s, clear 0.5 s, jam after 0.25 s,
    // unjam 0.3 s, alert after 3 jams in 5 s.
    private final FeedLogic m_logic = new FeedLogic(3, 0.25, 0.5, 0.25, 0.3, 3, 5.0);

    private Out[] at(double t, Mode mode) {
        return m_logic.update(t, mode, kFine);
    }

    @Test
    void feedRunsEveryStage() {
        assertArrayEquals(new Out[] {F, F, F}, at(0, Mode.FEED));
    }

    @Test
    void stoppedFromTheStartBrakesEveryStage() {
        assertArrayEquals(new Out[] {B, B, B}, at(0, Mode.STOP));
    }

    @Test
    void stopKeepsTheLastStageRunningBriefly() {
        // The ball at the last stage goes on into the flywheel instead of being held against it.
        at(0, Mode.FEED);
        at(0.10, Mode.FEED);
        assertArrayEquals(new Out[] {B, B, F}, at(0.12, Mode.STOP));
        assertArrayEquals(new Out[] {B, B, F}, at(0.30, Mode.STOP));
        assertArrayEquals(new Out[] {B, B, B}, at(0.40, Mode.STOP));
    }

    @Test
    void aStallLongerThanTheJamTimeReversesEveryStageThenFeedsAgain() {
        boolean[] stuck = {false, true, false};
        assertArrayEquals(new Out[] {F, F, F}, m_logic.update(0.00, Mode.FEED, stuck));
        assertArrayEquals(new Out[] {F, F, F}, m_logic.update(0.20, Mode.FEED, stuck));
        assertArrayEquals(new Out[] {R, R, R}, m_logic.update(0.26, Mode.FEED, stuck));
        assertEquals(1, m_logic.jammedStage());
        assertEquals(1, m_logic.recentJams(0.26));
        // Still reversing, whatever the stall reads.
        assertArrayEquals(new Out[] {R, R, R}, m_logic.update(0.50, Mode.FEED, kFine));
        assertArrayEquals(new Out[] {F, F, F}, m_logic.update(0.60, Mode.FEED, kFine));
    }

    @Test
    void aShortStallIsNotAJam() {
        boolean[] stuck = {true, false, false};
        m_logic.update(0.00, Mode.FEED, stuck);
        m_logic.update(0.20, Mode.FEED, stuck);
        assertArrayEquals(new Out[] {F, F, F}, m_logic.update(0.22, Mode.FEED, kFine));
        assertArrayEquals(new Out[] {F, F, F}, m_logic.update(0.40, Mode.FEED, stuck), "the stall time starts again");
        assertEquals(0, m_logic.recentJams(0.40));
    }

    @Test
    void afterAnUnjamTheJamCheckWaitsItsFullTimeAgain() {
        boolean[] stuck = {false, false, true};
        m_logic.update(0.00, Mode.FEED, stuck);
        m_logic.update(0.26, Mode.FEED, stuck); // jam, reverse until 0.56
        assertArrayEquals(new Out[] {F, F, F}, m_logic.update(0.58, Mode.FEED, stuck));
        assertArrayEquals(new Out[] {F, F, F}, m_logic.update(0.80, Mode.FEED, stuck));
        assertArrayEquals(new Out[] {R, R, R}, m_logic.update(0.84, Mode.FEED, stuck));
        assertEquals(2, m_logic.recentJams(0.84));
    }

    @Test
    void repeatedJamsRaiseTheAlertUntilTheyAge() {
        boolean[] stuck = {true, false, false};
        double t = 0;
        for (int i = 0; i < 3; i++) {
            m_logic.update(t, Mode.FEED, stuck);
            m_logic.update(t + 0.26, Mode.FEED, stuck);
            m_logic.update(t + 0.60, Mode.FEED, kFine);
            t += 1.0;
        }
        assertTrue(m_logic.jamAlert(t));
        assertFalse(m_logic.jamAlert(t + 6.0));
    }

    @Test
    void stopWhileUnjammingBrakesWithNoRunOn() {
        boolean[] stuck = {false, true, false};
        m_logic.update(0.00, Mode.FEED, stuck);
        m_logic.update(0.26, Mode.FEED, stuck);
        assertArrayEquals(new Out[] {B, B, B}, at(0.30, Mode.STOP));
    }

    @Test
    void ejectReversesEveryStageAndLeavesNoRunOn() {
        at(0, Mode.FEED);
        assertArrayEquals(new Out[] {R, R, R}, at(0.10, Mode.EJECT));
        assertArrayEquals(new Out[] {B, B, B}, at(0.12, Mode.STOP));
    }

    @Test
    void noJamCheckWhileEjecting() {
        boolean[] stuck = {true, true, true};
        m_logic.update(0.0, Mode.EJECT, stuck);
        m_logic.update(1.0, Mode.EJECT, stuck);
        assertEquals(0, m_logic.recentJams(1.0));
    }

    @Test
    void endAfterAFeedRunsOnThenReversesEveryStageThenBrakes() {
        // The ball at the last stage goes into the wheel; then the rest go back from the turret.
        at(0, Mode.FEED);
        at(0.10, Mode.FEED); // run-on until 0.35
        assertArrayEquals(new Out[] {B, B, F}, at(0.12, Mode.END));
        assertArrayEquals(new Out[] {B, B, F}, at(0.30, Mode.END));
        assertArrayEquals(new Out[] {R, R, R}, at(0.40, Mode.END));
        assertTrue(m_logic.isClearing(0.40));
        assertArrayEquals(new Out[] {R, R, R}, at(0.80, Mode.END));
        assertArrayEquals(new Out[] {B, B, B}, at(0.90, Mode.END));
        assertFalse(m_logic.isClearing(0.90));
    }

    @Test
    void endAfterAPauseReversesAtOnce() {
        at(0, Mode.FEED);
        assertArrayEquals(new Out[] {B, B, F}, at(0.10, Mode.STOP), "a pause runs on, as before");
        assertArrayEquals(new Out[] {B, B, B}, at(0.50, Mode.STOP), "and then brakes, with no reverse");
        assertArrayEquals(new Out[] {R, R, R}, at(0.60, Mode.END));
        assertArrayEquals(new Out[] {R, R, R}, at(1.05, Mode.END));
        assertArrayEquals(new Out[] {B, B, B}, at(1.15, Mode.END));
    }

    @Test
    void endWithNothingFedBrakes() {
        at(0, Mode.STOP);
        assertArrayEquals(new Out[] {B, B, B}, at(0.10, Mode.END));
        assertArrayEquals(new Out[] {B, B, B}, at(0.50, Mode.END));
    }

    @Test
    void endDuringAnUnjamReversesForTheClearTime() {
        boolean[] stuck = {false, true, false};
        m_logic.update(0.00, Mode.FEED, stuck);
        m_logic.update(0.26, Mode.FEED, stuck); // unjam until 0.56
        assertArrayEquals(new Out[] {R, R, R}, at(0.30, Mode.END));
        assertArrayEquals(new Out[] {R, R, R}, at(0.75, Mode.END));
        assertArrayEquals(new Out[] {B, B, B}, at(0.85, Mode.END));
    }

    @Test
    void aSecondEndWithNoFeedBetweenDoesNotReverseAgain() {
        at(0, Mode.FEED);
        at(0.10, Mode.END);
        at(0.90, Mode.END); // clear done
        at(1.00, Mode.STOP); // a new shot that never gets ready
        assertArrayEquals(new Out[] {B, B, B}, at(1.10, Mode.END));
    }

    @Test
    void aFeedInTheMiddleOfAClearFeeds() {
        at(0, Mode.FEED);
        at(0.40, Mode.END); // reverse from 0.40
        assertArrayEquals(new Out[] {R, R, R}, at(0.50, Mode.STOP), "a new shot, not ready yet: the clear goes on");
        assertArrayEquals(new Out[] {F, F, F}, at(0.60, Mode.FEED));
        assertFalse(m_logic.isClearing(0.60));
    }

    @Test
    void ejectLeavesNothingToClear() {
        at(0, Mode.FEED);
        at(0.10, Mode.EJECT);
        assertArrayEquals(new Out[] {B, B, B}, at(0.50, Mode.END));
    }
}
