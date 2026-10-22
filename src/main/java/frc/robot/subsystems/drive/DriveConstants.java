package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

/** Driver feel and control values for the swerve. The hardware values are in TunerConstants. */
public final class DriveConstants {
    private DriveConstants() {}

    /**
     * Full stick speed: 90 % of the free speed at 12 V. Closed loop cannot reach 100 %: under load
     * and at 11 V the motors run out of voltage, and then a turn asked at full speed is lost.
     */
    public static final double kMaxSpeed = 0.9 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Full stick turn rate, rad/s. */
    public static final double kMaxAngularRate = RotationsPerSecond.of(1.85).in(RadiansPerSecond);

    /** Stick values below this are 0. */
    public static final double kStickDeadband = 0.08;
    /** Squared: fine control at low speed for driving. */
    public static final double kTranslationExponent = 2;
    /** Cubed: finer control for turning, where small errors show most. */
    public static final double kRotationExponent = 3;

    /**
     * Heading controller, rad/s of turn per radian of heading error. Used to hold the heading,
     * snap to an angle, and face the hub. 6 means a 10 degree error asks for about 1 rad/s.
     * Tune with the "Rotation" SysId routine.
     */
    public static final double kHeadingP = 6.0;
    /** The robot must turn slower than this (rad/s) before heading hold takes the heading. */
    public static final double kHeadingHoldRateThreshold = 0.15;
    /** Heading hold only runs while the robot moves faster than this (m/s). */
    public static final double kHeadingHoldMinSpeed = 0.05;

    /** Distance from the robot center to a wheel, meters (for the wheel-radius test). */
    public static final double kDriveBaseRadius =
        Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);

    /** Wheel-radius test: turn rate and how fast to ramp up to it. */
    public static final double kWheelRadiusMaxVelocity = 0.5; // rad/s
    public static final double kWheelRadiusRampRate = 0.25; // rad/s^2
}
