package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import frc.robot.calibration.ShotPoints.ShotPoint;

class CalibrationStoreTest {
    @TempDir
    Path m_dir;

    private static CalibrationData sample() {
        var data = new CalibrationData();
        data.shotPoints = List.of(new ShotPoint(2.0, 81.5, 15.25, Double.NaN), new ShotPoint(3.0, 90.0, 22.0, 0.8));
        data.steerOffsets = new double[] {-0.2, -0.474, 0.3723144, 0.118};
        data.turretTrimDeg = 1.25;
        data.intakeOutStopLeft = 0.326;
        data.intakeOutStopRight = 0.331;
        data.hoodRatio = 86.5;
        data.feedSpeeds = new double[] {96, 125, 112.5, 100.4, 91.5};
        return data;
    }

    @Test
    void aMissingFileGivesNoValues() throws IOException {
        var data = new CalibrationStore(m_dir.resolve("none.json")).load();
        assertTrue(data.shotPoints.isEmpty());
        assertNull(data.steerOffsets);
        assertTrue(Double.isNaN(data.hoodRatio));
    }

    @Test
    void whatIsSavedLoadsBack() throws IOException {
        var store = new CalibrationStore(m_dir.resolve("calibration.json"));
        assertNull(store.save(sample()), "no old file, no backup");
        var data = store.load();
        assertEquals(sample().shotPoints, data.shotPoints);
        assertTrue(Double.isNaN(data.shotPoints.get(0).timeOfFlight()));
        assertArrayEquals(sample().steerOffsets, data.steerOffsets);
        assertEquals(1.25, data.turretTrimDeg);
        assertEquals(0.326, data.intakeOutStopLeft);
        assertEquals(0.331, data.intakeOutStopRight);
        assertEquals(86.5, data.hoodRatio);
        assertTrue(Double.isNaN(data.hoodStopAboveLevelDeg), "not measured stays not measured");
        assertArrayEquals(sample().feedSpeeds, data.feedSpeeds);
    }

    @Test
    void aSaveKeepsTheOldFile() throws IOException {
        var store = new CalibrationStore(m_dir.resolve("calibration.json"));
        store.save(sample());
        var changed = sample();
        changed.hoodRatio = 90;
        Path backup = store.save(changed);
        assertEquals(86.5, new CalibrationStore(backup).load().hoodRatio);
        assertEquals(90, store.load().hoodRatio);
        assertEquals(m_dir.resolve("calibration-backup"), backup.getParent());
        assertTrue(Files.notExists(m_dir.resolve("calibration.json.tmp")));
    }

    @Test
    void aBrokenFileIsAnError() throws IOException {
        Path path = m_dir.resolve("calibration.json");
        Files.writeString(path, "{\"hoodRatio\": 86.5,");
        assertThrows(IllegalArgumentException.class, () -> new CalibrationStore(path).load());
    }

    @Test
    void unknownKeysAreIgnored() throws IOException {
        Path path = m_dir.resolve("calibration.json");
        Files.writeString(path, "{\"note\": \"pit, 10-22\", \"hoodRatio\": 84, \"old\": [1, true, null, {}]}");
        assertEquals(84, new CalibrationStore(path).load().hoodRatio);
    }
}
