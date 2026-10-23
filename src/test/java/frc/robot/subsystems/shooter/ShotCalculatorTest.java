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

    /** How far the ball drifts per m/s of shooter velocity for a flight of {@code tof} s (drag slows the drift). */
    private static double drift(double tof) {
        return tof * (1 - ShooterConstants.kLeadDecay * tof);
    }

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
    void drivingHomeAimsAtTheHubBeforeTheZone() {
        // 1 m outside our zone, driving home at 2 m/s: in the zone within 0.5 s.
        var robot = new Pose2d(FieldConstants.k_blueZoneTransition + 1.0, 2.5, Rotation2d.ZERO);
        var home = new ChassisVelocities(-2.0, 0, 0);
        assertFalse(m_calc.calculate(robot, home, 0, false, 0, 0).scoring(), "no look ahead: still a pass");
        var shot = m_calc.calculate(robot, home, 0, false, 0, 1.0);
        assertTrue(shot.scoring());
        assertEquals(FieldConstants.hub(false), shot.target());
        assertFalse(m_calc.calculate(robot, new ChassisVelocities(2.0, 0, 0), 0, false, 0, 1.0).scoring(),
            "driving away from home: still a pass");
    }

    @Test
    void redRobotAimsAtTheRedHub() {
        var shot = m_calc.calculate(new Pose2d(14.0, 4.0, Rotation2d.k180deg), kStill, 0, true, 0);
        assertTrue(shot.scoring());
        assertEquals(FieldConstants.hub(true), shot.target());
    }

    @Test
    void turningRobotLeadsTheGoalButNotTheReadyAngle() {
        // Robot turning +1 rad/s, no latency: the goal leads by (bearing rate - turn rate) x the
        // lead time, clockwise relative to the robot; the angle for "ready" is the true one. The
        // turret axis is off the robot center, so the turn also moves it (a small bearing rate).
        var target = new Translation2d(5, 5);
        var aim = ShotCalculator.aim(robotWithPivotAt(target.minus(new Translation2d(0, 2))), target, kStill, 1.0, kNoFlight, 0, 0);
        double pivotVx = -1.0 * ShooterConstants.kTurretOffset.getY();
        double bearingRate = 2 * pivotVx / 4; // (r_y v_x - r_x v_y) / r^2, r = (0, 2)
        assertEquals(0.0, aim.turretAimRot(), 1e-9);
        assertEquals((bearingRate - 1.0) * ShooterConstants.kTurretLeadTime / (2 * Math.PI), aim.turretRot(), 1e-9);
    }

    @Test
    void aTurningRobotsBallLeavesWithTheTurretAxisVelocity() {
        // Still robot turning at 2 rad/s, 1 s flight: the turret axis moves at omega x its offset,
        // so the aim point is moved back by that velocity x the drift time.
        var oneSecond = new ShotTable(new double[][] {{0, 1.0}});
        var target = new Translation2d(5, 5);
        var aim = ShotCalculator.aim(robotWithPivotAt(target.minus(new Translation2d(0, 3))), target, kStill, 2.0, oneSecond, 0, 0);
        var offset = ShooterConstants.kTurretOffset;
        assertEquals(target.getX() + 2.0 * offset.getY() * drift(1.0), aim.aimPoint().getX(), 1e-9);
        assertEquals(target.getY() - 2.0 * offset.getX() * drift(1.0), aim.aimPoint().getY(), 1e-9);
    }

    @Test
    void latencyShootsFromWhereTheRobotWillBe() {
        // Driving +x at 2 m/s straight at a target 3 m ahead, the ball leaves 0.1 s later: from
        // 0.2 m nearer. (No flight time, so no aim lead.)
        var target = new Translation2d(5, 5);
        var aim = ShotCalculator.aim(robotWithPivotAt(target.minus(new Translation2d(3, 0))), target,
            new ChassisVelocities(2, 0, 0), 0, kNoFlight, 0, 0.1);
        assertEquals(2.8, aim.distance(), 1e-9);
    }

    @Test
    void theLeadIsSolvedForAFlightTimeThatDependsOnTheDistance() {
        // Time of flight 0.2 s per meter: the aim point must satisfy aim = target - v x drift(tof(aim)).
        var tof = new ShotTable(new double[][] {{0, 0}, {10, 2}});
        var target = new Translation2d(5, 5);
        var pivot = target.minus(new Translation2d(4, 0));
        var v = new ChassisVelocities(1.5, -1.0, 0);
        var aim = ShotCalculator.aim(robotWithPivotAt(pivot), target, v, 0, tof, 0, 0);
        double t = drift(tof.get(aim.aimPoint().getDistance(pivot)));
        assertEquals(target.getX() - 1.5 * t, aim.aimPoint().getX(), 1e-3);
        assertEquals(target.getY() + 1.0 * t, aim.aimPoint().getY(), 1e-3);
    }

    @Test
    void strafingPastTheTargetLeadsTheTurretByTheBearingRate() {
        // Target 3 m to the robot's left, robot moving +x at 2 m/s: the target's bearing turns at
        // 2/3 rad/s, so the goal leads the aim by that x the lead time.
        var target = new Translation2d(5, 5);
        var aim = ShotCalculator.aim(robotWithPivotAt(target.minus(new Translation2d(0, 3))), target,
            new ChassisVelocities(2, 0, 0), 0, kNoFlight, 0, 0);
        assertEquals(2.0 / 3 * ShooterConstants.kTurretLeadTime / (2 * Math.PI), aim.turretRot() - aim.turretAimRot(), 1e-9);
    }

    @Test
    void drivingTowardTheHubLeadsTheHoodAndFlywheelToTheNearerDistance() {
        // In our zone, 5 m out, driving straight at the hub at 1 m/s: the goals are for the
        // distance a lead time ahead, the "ready" values for the distance now.
        var hub = FieldConstants.hub(false);
        var pivot = hub.minus(new Translation2d(5.0, 0));
        var robot = robotWithPivotAt(pivot);
        var shot = m_calc.calculate(robot, new ChassisVelocities(1, 0, 0), 0, false, 0);
        assertTrue(shot.distanceRate() < -0.9, "closing at about 1 m/s");
        double d = shot.distance();
        assertEquals(ShooterConstants.kScoreSpeeds.get(d), shot.flywheelRps(), 1e-9);
        assertEquals(ShooterConstants.kScoreSpeeds.get(d + shot.distanceRate() * ShooterConstants.kFlywheelLeadTime),
            shot.flywheelGoalRps(), 1e-9);
        assertEquals(ShooterConstants.kScoreHood.get(d + shot.distanceRate() * ShooterConstants.kHoodLeadTime),
            shot.hoodGoalDeg(), 1e-9);
        assertTrue(shot.flywheelGoalRps() < shot.flywheelRps(), "nearer = slower");
    }

    @Test
    void turretToleranceIsASideErrorAtTheHub() {
        assertEquals(Math.toDegrees(Math.atan(ShooterConstants.kTurretToleranceMeters / 3.0)),
            ShotCalculator.turretToleranceDeg(3.0), 1e-9);
        assertEquals(ShooterConstants.kTurretMinToleranceDeg, ShotCalculator.turretToleranceDeg(50.0), 1e-9);
    }

    @Test
    void shootOnTheMoveOffAimsAsIfStill() {
        var robot = new Pose2d(2.5, 3.0, Rotation2d.ZERO);
        var moving = new ChassisVelocities(1.5, 1.0, 0);
        m_calc.setShootOnTheMove(false);
        var off = m_calc.calculate(robot, moving, 0, false, 0);
        var still = m_calc.calculate(robot, kStill, 0, false, 0);
        assertEquals(still.distance(), off.distance(), 1e-9);
        assertEquals(still.turretAimRot(), off.turretAimRot(), 1e-9);
        m_calc.setShootOnTheMove(true);
        assertTrue(Math.abs(m_calc.calculate(robot, moving, 0, false, 0).distance() - still.distance()) > 0.1);
    }

    @Test
    void movingRobotLeadsAgainstItsVelocity() {
        // 1 s time of flight, robot moving +x at 1 m/s: aim at a point drift(1 s) behind the target in x.
        var oneSecond = new ShotTable(new double[][] {{0, 1.0}});
        var target = new Translation2d(5, 5);
        var aim = ShotCalculator.aim(robotWithPivotAt(target.minus(new Translation2d(0, 3))), target,
            new ChassisVelocities(1, 0, 0), 0, oneSecond, 0, 0);
        // The aim point is at smaller x, so the turret turns a little left of the jig angle.
        assertTrue(aim.turretRot() > 0, "lead should turn the turret toward smaller x");
        assertEquals(Math.hypot(drift(1.0), 3), aim.distance(), 1e-6);
    }
}
