package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.List;

import org.junit.jupiter.api.Test;

import frc.robot.calibration.ShotPoints.ShotPoint;
import frc.robot.subsystems.shooter.ShotTable;

class ShotPointsTest {
    private static final ShotTable kConstants = new ShotTable(new double[][] {{1.0, 10}, {2.0, 20}, {3.0, 30}, {4.0, 40}});

    @Test
    void pointsStaySortedByDistance() {
        var points = ShotPoints.add(List.of(), new ShotPoint(3.0, 90, 22, Double.NaN));
        points = ShotPoints.add(points, new ShotPoint(2.0, 80, 15, Double.NaN));
        assertEquals(2.0, points.get(0).distance());
        assertEquals(3.0, points.get(1).distance());
    }

    @Test
    void theSameSpotAgainReplacesThePoint() {
        var points = ShotPoints.add(List.of(new ShotPoint(2.0, 80, 15, Double.NaN)), new ShotPoint(2.05, 82, 16, Double.NaN));
        assertEquals(1, points.size());
        assertEquals(82, points.get(0).flywheelRps());
    }

    @Test
    void deleteTakesTheNearestPoint() {
        var points = List.of(new ShotPoint(2.0, 80, 15, Double.NaN), new ShotPoint(3.0, 90, 22, Double.NaN));
        var left = ShotPoints.deleteNearest(points, 2.8);
        assertEquals(1, left.size());
        assertEquals(2.0, left.get(0).distance());
        assertEquals(0, ShotPoints.deleteNearest(List.of(), 2.0).size());
        assertNull(ShotPoints.nearest(List.of(), 2.0));
    }

    @Test
    void noRecordedPointsGiveTheConstants() {
        var table = ShotPoints.merge(kConstants, List.of(), ShotPoint::flywheelRps);
        assertEquals(25, table.get(2.5), 1e-9);
    }

    @Test
    void aRecordedPointReplacesTheConstantsNearIt() {
        var table = ShotPoints.merge(kConstants, List.of(new ShotPoint(2.1, 50, 15, Double.NaN)), ShotPoint::flywheelRps);
        assertEquals(50, table.get(2.1), 1e-9);
        assertEquals(10 + 40 / 1.1, table.get(2.0), 1e-9, "the constant at 2.0 is gone: between 1.0 and 2.1 now");
        assertEquals(10, table.get(1.0), 1e-9, "far constants stay");
        assertEquals(30, table.get(3.0), 1e-9);
    }

    @Test
    void onlyPointsWithATimeOfFlightChangeThatTable() {
        var points = List.of(new ShotPoint(2.0, 80, 15, Double.NaN), new ShotPoint(3.0, 90, 22, 0.9));
        var tof = ShotPoints.merge(kConstants, points, ShotPoint::timeOfFlight);
        assertEquals(20, tof.get(2.0), 1e-9);
        assertEquals(0.9, tof.get(3.0), 1e-9);
    }
}
