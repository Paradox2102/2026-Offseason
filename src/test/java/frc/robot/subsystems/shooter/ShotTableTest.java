package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ShotTableTest {
    private final ShotTable m_table = new ShotTable(new double[][] {{1.0, 10}, {2.0, 20}, {4.0, 30}});

    @Test
    void interpolatesBetweenPoints() {
        assertEquals(15, m_table.get(1.5), 1e-9);
        assertEquals(25, m_table.get(3.0), 1e-9);
    }

    @Test
    void clampsAtTheEnds() {
        assertEquals(10, m_table.get(0.2), 1e-9);
        assertEquals(30, m_table.get(9.0), 1e-9);
    }

    @Test
    void scalesValues() {
        assertEquals(30, m_table.scaled(2.0).get(1.5), 1e-9);
    }
}
