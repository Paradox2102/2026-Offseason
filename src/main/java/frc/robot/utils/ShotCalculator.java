package frc.robot.utils;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.telemetry.Telemetry;

import frc.robot.Constants.ShooterConstants;


public class ShotCalculator {

    /**
     * The result of calculate()
     * @param turretAngle Angle to aim the shooter at, relative to the robot's front (same frame as PhotonUtils.getYawToPose)
     * @param turretVelocityRadPerSec How fast turretAngle is changing, use as a velocity feedforward
     * @param projectedDistance Distance to the virtual hub, use this for the hood and shooter lookup tables
     * @param timeOfFlight Solved time of flight in seconds
     * @param iterations Newton iterations used, more than k_maxIterations means it diverged and fell back to a static shot
     */
    public record LaunchParameters(
        Rotation2d turretAngle,
        double turretVelocityRadPerSec,
        double projectedDistance,
        double timeOfFlight,
        int iterations,
        boolean isValid) {

        public static final LaunchParameters INVALID = new LaunchParameters(new Rotation2d(), 0, 0, 0, 0, false);
    }

    // Newton solver internals, these shouldn't need tuning
    private static final int k_maxIterations = 25;
    private static final double k_convergenceTolerance = 0.001; // seconds
    private static final double k_tofMax = 5.0; // seconds
    private static final double k_derivativeStep = 0.01; // meters
    private static final double k_loopTime = 0.02;

    private final InterpolatingDoubleTreeMap m_tofMap = new InterpolatingDoubleTreeMap();
    private final String m_logPath;

    private double m_previousTOF = -1;
    private double m_prevVx = 0;
    private double m_prevVy = 0;
    private double m_prevOmega = 0;
    private boolean m_hasPrevVelocity = false;
    /** Operator adjustment added to k_sotmLatency mid-match */
    private double m_latencyOffset = 0;

    /**
     * @param shotTimes Lookup table of {distance meters, time of flight seconds}
     * @param name Name used for telemetry
     */
    public ShotCalculator(double[][] shotTimes, String name) {
        for (double[] values : shotTimes) {
            m_tofMap.put(values[0], values[1]);
        }
        m_logPath = "SOTM/" + name + "/";
    }

    /** @return k_sotmLatency plus the operator's adjustment, in seconds */
    public double getLatency() {
        return ShooterConstants.k_sotmLatency + m_latencyOffset;
    }

    /**
     * Changes the latency mid-match, it can't go below 0
     * @param delta seconds to add, negative to decrease
     */
    public void adjustLatency(double delta) {
        m_latencyOffset = Math.max(-ShooterConstants.k_sotmLatency, m_latencyOffset + delta);
    }

    private double getTOF(double distance) {
        return m_tofMap.get(distance);
    }

    /** Central difference derivative of the time of flight table */
    private double getTOFDerivative(double distance) {
        return (getTOF(distance + k_derivativeStep) - getTOF(distance - k_derivativeStep)) / (2 * k_derivativeStep);
    }

    /** Weird math I don't fully understand. Calculates drag mid-air for fuel. */
    private double getDriftTime(double tof) {
        double c = ShooterConstants.k_sotmDragCoeff;
        if (c < 1e-6) return tof;
        return (1 - Math.exp(-c * tof)) / c;
    }

    /**
     * Solves for the shot, call once per loop
     * @param robotPose Field relative pose of the robot
     * @param fieldVelocity Field relative velocity of the robot
     * @param target Field position to score into
     */
    public LaunchParameters calculate(Pose2d robotPose, ChassisVelocities fieldVelocity, Translation2d target) {
        if (Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY())
            || Double.isInfinite(robotPose.getX()) || Double.isInfinite(robotPose.getY())) {
            return LaunchParameters.INVALID;
        }

        // Predict where the robot will be when the fuel leaves, using v*dt + 0.5*a*dt^2
        double dt = getLatency();

        double ax = m_hasPrevVelocity ? (fieldVelocity.vx - m_prevVx) / k_loopTime : 0;
        double ay = m_hasPrevVelocity ? (fieldVelocity.vy - m_prevVy) / k_loopTime : 0;
        double aOmega = m_hasPrevVelocity ? (fieldVelocity.omega - m_prevOmega) / k_loopTime : 0;
        m_hasPrevVelocity = true;
        m_prevVx = fieldVelocity.vx;
        m_prevVy = fieldVelocity.vy;
        m_prevOmega = fieldVelocity.omega;

        double robotX = robotPose.getX() + fieldVelocity.vx * dt + 0.5 * ax * dt * dt;
        double robotY = robotPose.getY() + fieldVelocity.vy * dt + 0.5 * ay * dt * dt;
        Rotation2d heading = robotPose.getRotation().plus(new Rotation2d(fieldVelocity.omega * dt + 0.5 * aOmega * dt * dt));

        Translation2d launcherOffset = ShooterConstants.k_turretOffset.rotateBy(heading);
        double launcherX = robotX + launcherOffset.getX();
        double launcherY = robotY + launcherOffset.getY();

        double omega = fieldVelocity.omega;
        double vx = fieldVelocity.vx - launcherOffset.getY() * omega;
        double vy = fieldVelocity.vy + launcherOffset.getX() * omega;
        double speed = Math.hypot(vx, vy);

        if (speed > ShooterConstants.k_sotmMaxSpeed) {
            return LaunchParameters.INVALID;
        }

        double rx = target.getX() - launcherX;
        double ry = target.getY() - launcherY;
        double distance = Math.hypot(rx, ry);

        double tof;
        double projectedDistance = distance;
        int iterations = 0;
        boolean isMoving = speed >= ShooterConstants.k_sotmMinSpeed;

        if (!isMoving) {
            tof = getTOF(distance);
        } else {
            // Start from last loop's answer since it's usually really close
            tof = m_previousTOF > 0 ? m_previousTOF : getTOF(distance);

            for (int i = 0; i < k_maxIterations; i++) {
                double prevTOF = tof;

                double dragExp = ShooterConstants.k_sotmDragCoeff < 1e-6 ? 1 : Math.exp(-ShooterConstants.k_sotmDragCoeff * tof);
                double driftTime = getDriftTime(tof);

                double prx = rx - vx * driftTime;
                double pry = ry - vy * driftTime;
                projectedDistance = Math.hypot(prx, pry);

                // The fuel would basically land on the hub on its own
                if (projectedDistance < 0.01) {
                    iterations = k_maxIterations + 1;
                    break;
                }

                double f = getTOF(projectedDistance) - tof;
                double dDistance = -dragExp * (prx * vx + pry * vy) / projectedDistance;
                double fPrime = getTOFDerivative(projectedDistance) * dDistance - 1;

                if (Math.abs(fPrime) > 0.01) {
                    tof = tof - f / fPrime;
                } else {
                    tof = getTOF(projectedDistance);
                }
                tof = Math.max(0, Math.min(k_tofMax, tof));

                iterations = i + 1;
                if (Math.abs(tof - prevTOF) < k_convergenceTolerance) {
                    break;
                }
            }

            // fall back to a static shot
            if (iterations > k_maxIterations || Double.isNaN(tof)) {
                isMoving = false;
                tof = getTOF(distance);
                projectedDistance = distance;
            }
        }
        m_previousTOF = tof;

        double aimX = rx;
        double aimY = ry;
        double turretVelocity = 0;
        if (isMoving) {
            double driftTime = getDriftTime(tof);
            aimX -= vx * driftTime;
            aimY -= vy * driftTime;

            if (distance > 0.1) {
                turretVelocity = (ry * vx - rx * vy) / (distance * distance) - omega;
            }
        }
        Rotation2d turretAngle = new Rotation2d(aimX, aimY).minus(heading);
        

        // Logging, feel free to comment out.
        Telemetry.log(m_logPath + "Time Of Flight", tof);
        Telemetry.log(m_logPath + "Distance", distance);
        Telemetry.log(m_logPath + "Projected Distance", projectedDistance);
        Telemetry.log(m_logPath + "Turret Angle Degrees", turretAngle.getDegrees());
        Telemetry.log(m_logPath + "Turret Velocity FF", turretVelocity);
        Telemetry.log(m_logPath + "Iterations", iterations);
        Telemetry.log(m_logPath + "Virtual Target X", launcherX + aimX);
        Telemetry.log(m_logPath + "Virtual Target Y", launcherY + aimY);

        return new LaunchParameters(turretAngle, turretVelocity, projectedDistance, tof, iterations, true);
    }

    public void reset() {
        m_previousTOF = -1;
        m_prevVx = 0;
        m_prevVy = 0;
        m_prevOmega = 0;
        m_hasPrevVelocity = false;
    }
}
