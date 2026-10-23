package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;

class HeadingReferenceTest {
    private final HeadingReference m_reference = new HeadingReference();

    private static Rotation2d deg(double degrees) {
        return Rotation2d.fromDegrees(degrees);
    }

    @Test
    void noObservationKeepsTheEstimator() {
        assertEquals(37.0, m_reference.correct(10.0, deg(37)).getDegrees(), 1e-9);
    }

    @Test
    void largeOffsetIsApplied() {
        // After a Pigeon reboot: multi-tag says 0 degrees, the estimator says 90.
        m_reference.observe(10.0, deg(0), deg(90));
        assertEquals(5.0, m_reference.correct(10.5, deg(95)).getDegrees(), 1e-9);
    }

    @Test
    void smallOffsetIsIgnored() {
        // A few degrees: unchecked camera angles, not a wrong gyro. The gyro is better.
        m_reference.observe(10.0, deg(0), deg(5));
        assertEquals(5.0, m_reference.correct(10.5, deg(5)).getDegrees(), 1e-9);
    }

    @Test
    void oldObservationExpires() {
        m_reference.observe(10.0, deg(0), deg(90));
        assertEquals(90.0, m_reference.correct(11.5, deg(90)).getDegrees(), 1e-9);
    }
}
