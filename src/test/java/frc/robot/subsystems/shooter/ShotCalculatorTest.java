package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import frc.robot.constants.FieldConstants;

class ShotCalculatorTest {
    private static final ChassisVelocities kStill = new ChassisVelocities();
    private final ShotCalculator m_calc = new ShotCalculator();

    /** A robot facing +x whose turret pivot is at {@code pivot}. */
    private static Pose2d robotWithPivotAt(Translation2d pivot) {
        return new Pose2d(pivot.minus(ShooterConstants.kTurretOffset), Rotation2d.ZERO);
    }

    private static final ShotTable kNoFlight = new ShotTable(new double[][] {{0, 0}});

    @Test
    void targetStraightLeftIsTheJigAngle() {
        var target = new Translation2d(5, 5);
        var aim = ShotCalculator.aim(robotWithPivotAt(target.minus(new Translation2d(0, 2))), target, kStill, 0, kNoFlight, 0);
        assertEquals(0.0, aim.turretRot(), 1e-9);
        assertEquals(2.0, aim.distance(), 1e-9);
    }

    @Test
    void targetStraightAheadIsAQuarterTurnRight() {
        var target = new Translation2d(5, 5);
        var aim = ShotCalculator.aim(robotWithPivotAt(target.minus(new Translation2d(2, 0))), target, kStill, 0, kNoFlight, 0);
        assertEquals(-0.25, aim.turretRot(), 1e-9);
    }

    @Test
    void inOurZoneItScoresWithTheTables() {
        var robot = new Pose2d(2.0, 3.0, Rotation2d.ZERO);
        var shot = m_calc.calculate(robot, kStill, 0, false, 0);
        assertTrue(shot.scoring());
        assertEquals(FieldConstants.hub(false), shot.target());
        assertEquals(ShooterConstants.kScoreSpeeds.get(shot.distance()), shot.flywheelRps(), 1e-9);
        assertEquals(ShooterConstants.kScoreHood.get(shot.distance()), shot.hoodDeg(), 1e-9);
    }

    @Test
    void outsideOurZoneItPasses() {
        // Mid-field, low y: the blue outpost.
        var shot = m_calc.calculate(new Pose2d(8.0, 2.0, Rotation2d.ZERO), kStill, 0, false, 0);
        assertFalse(shot.scoring());
        assertEquals(FieldConstants.k_blueOutpost, shot.target());
    }

    @Test
    void redRobotAimsAtTheRedHub() {
        var shot = m_calc.calculate(new Pose2d(14.0, 4.0, Rotation2d.k180deg), kStill, 0, true, 0);
        assertTrue(shot.scoring());
        assertEquals(FieldConstants.hub(true), shot.target());
    }

    @Test
    void movingRobotLeadsAgainstItsVelocity() {
        // 1 s time of flight, robot moving +x at 1 m/s: aim at a point 1 m behind the target in x.
        var oneSecond = new ShotTable(new double[][] {{0, 1.0}});
        var target = new Translation2d(5, 5);
        var aim = ShotCalculator.aim(robotWithPivotAt(target.minus(new Translation2d(0, 3))), target,
            new ChassisVelocities(1, 0, 0), 0, oneSecond, 0);
        // The aim point is at smaller x, so the turret turns a little left of the jig angle.
        assertTrue(aim.turretRot() > 0, "lead should turn the turret toward smaller x");
        assertEquals(Math.hypot(1, 3), aim.distance(), 1e-6);
    }
}
