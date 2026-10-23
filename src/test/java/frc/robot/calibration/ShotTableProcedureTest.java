package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShotTableProcedureTest {
    @Test
    void aSmallMoveKeepsTheJoggedValues() {
        // Pose noise while the robot stands at a spot must not drop the jogs.
        assertFalse(ShotTableProcedure.movedFrom(2.0, 2.0));
        assertFalse(ShotTableProcedure.movedFrom(2.0, 2.09));
        assertFalse(ShotTableProcedure.movedFrom(2.0, 1.91));
    }

    @Test
    void aMovePastTheLimitLoadsTheTableAgain() {
        assertTrue(ShotTableProcedure.movedFrom(2.0, 2.11));
        assertTrue(ShotTableProcedure.movedFrom(3.0, 2.85), "closer counts too");
        assertTrue(ShotTableProcedure.movedFrom(2.42, 3.0), "a new typed distance");
    }
}
