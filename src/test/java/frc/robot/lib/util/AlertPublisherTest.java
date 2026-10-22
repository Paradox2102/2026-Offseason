package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.util.Alert.Level;

class AlertPublisherTest {
    @Test
    void createdAlertIsListedWhenSet() {
        var alert = AlertPublisher.create("TestA", "one", "Motor 3 is hot", Level.HIGH);
        assertArrayEquals(new String[0], AlertPublisher.active("TestA", Level.HIGH));
        alert.set(true);
        assertArrayEquals(new String[] {"Motor 3 is hot"}, AlertPublisher.active("TestA", Level.HIGH));
        alert.set(false);
        assertArrayEquals(new String[0], AlertPublisher.active("TestA", Level.HIGH));
    }

    @Test
    void levelsGoToTheirOwnList() {
        AlertPublisher.create("TestB", "high", "error text", Level.HIGH).set(true);
        AlertPublisher.create("TestB", "medium", "warning text", Level.MEDIUM).set(true);
        AlertPublisher.create("TestB", "low", "info text", Level.LOW).set(true);
        assertArrayEquals(new String[] {"error text"}, AlertPublisher.active("TestB", Level.HIGH));
        assertArrayEquals(new String[] {"warning text"}, AlertPublisher.active("TestB", Level.MEDIUM));
        assertArrayEquals(new String[] {"info text"}, AlertPublisher.active("TestB", Level.LOW));
    }
}
