package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class AimGateTest {
    private final AimGate m_gate = new AimGate();

    @Test
    void withNoFieldPoseTheTurretHoldsWhereItIs() {
        // Robot 2026-10-25: no vision pose, so the pose was the field corner, and the turret
        // turned almost 180 degrees away from the hub to a pass point 15.6 m off.
        assertEquals(0.0, m_gate.turretGoal(false, 0.41, 0.0), 1e-9);
        assertEquals(0.0, m_gate.turretGoal(false, 0.38, 0.02), 1e-9, "it keeps the first position, not a drift");
    }

    @Test
    void withAFieldPoseItAims() {
        m_gate.turretGoal(false, 0.41, 0.0);
        assertEquals(0.41, m_gate.turretGoal(true, 0.41, 0.0), 1e-9);
        assertEquals(0.30, m_gate.turretGoal(true, 0.30, 0.38), 1e-9);
    }

    @Test
    void aLaterLossHoldsTheNewPosition() {
        m_gate.turretGoal(true, 0.30, 0.30);
        assertEquals(0.29, m_gate.turretGoal(false, 0.5, 0.29), 1e-9);
    }

    @Test
    void withNoFieldPoseTheHoodStaysDown() {
        assertEquals(0.0, AimGate.hoodGoal(false, 22.5), 1e-9);
        assertEquals(22.5, AimGate.hoodGoal(true, 22.5), 1e-9);
    }
}
