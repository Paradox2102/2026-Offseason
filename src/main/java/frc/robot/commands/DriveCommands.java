package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.telemetry.Telemetry;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.TargetDirectionPerspectiveValue;

import frc.robot.constants.FieldConstants;
import frc.robot.lib.util.JoystickShaping;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.HeadingHold;

/**
 * Driver commands for the swerve.
 *
 * <p>Stick axes: WPILib's field +x is away from the driver and +y is to the driver's left, but a
 * stick reads -1 when pushed away and -1 when pushed left. So x = -leftY, y = -leftX and
 * turn = -rightX (counterclockwise is positive). The caller passes the raw sticks; the signs are
 * fixed here in one place.
 */
public final class DriveCommands {
    private DriveCommands() {}

    /** Shaped field speeds from the sticks: {vx m/s, vy m/s}. */
    private static double[] translation(DoubleSupplier leftY, DoubleSupplier leftX) {
        double x = JoystickShaping.shape(-leftY.getAsDouble(), DriveConstants.kStickDeadband, DriveConstants.kTranslationExponent);
        double y = JoystickShaping.shape(-leftX.getAsDouble(), DriveConstants.kStickDeadband, DriveConstants.kTranslationExponent);
        return new double[] {x * DriveConstants.kMaxSpeed, y * DriveConstants.kMaxSpeed};
    }

    private static SwerveRequest.FieldCentric fieldCentric() {
        return new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    private static SwerveRequest.FieldCentricFacingAngle facingAngle(TargetDirectionPerspectiveValue perspective) {
        return new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withTargetDirectionPerspective(perspective)
            .withMaxAbsRotationalRate(DriveConstants.kMaxAngularRate)
            .withHeadingPID(DriveConstants.kHeadingP, 0, 0);
    }

    /**
     * Teleop drive with heading hold.
     *
     * @param openLoop true = old open-loop voltage drive (a fallback for the first test of the
     *     closed-loop gains on the robot)
     */
    public static Command teleop(Drive drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX,
                                 BooleanSupplier openLoop) {
        var turning = fieldCentric();
        // The held heading is a field angle, so it must not be turned by the driver's perspective.
        var holding = facingAngle(TargetDirectionPerspectiveValue.BlueAlliance);
        var hold = new HeadingHold(DriveConstants.kHeadingHoldRateThreshold);

        return drive.run(() -> {
            double[] v = translation(leftY, leftX);
            double turn = JoystickShaping.shape(-rightX.getAsDouble(), DriveConstants.kStickDeadband, DriveConstants.kRotationExponent)
                * DriveConstants.kMaxAngularRate;
            var driveType = openLoop.getAsBoolean() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity;
            boolean translating = Math.hypot(v[0], v[1]) > DriveConstants.kHeadingHoldMinSpeed;

            var held = hold.update(turn, drive.getRobotVelocity().omega, drive.getHeading(), translating);
            if (held.isPresent()) {
                drive.setControl(holding.withVelocityX(v[0]).withVelocityY(v[1])
                    .withTargetDirection(held.get()).withDriveRequestType(driveType));
            } else {
                drive.setControl(turning.withVelocityX(v[0]).withVelocityY(v[1])
                    .withRotationalRate(turn).withDriveRequestType(driveType));
            }
        }).beforeStarting(hold::reset).withName("Teleop Drive");
    }

    /**
     * Drive with the sticks while the robot turns to a fixed angle, seen from the driver: 0 = away
     * from the driver, 90 = to the driver's left.
     */
    public static Command snapTo(Drive drive, DoubleSupplier leftY, DoubleSupplier leftX, Rotation2d target) {
        var request = facingAngle(TargetDirectionPerspectiveValue.UseForwardPerspective);
        return drive.run(() -> {
            double[] v = translation(leftY, leftX);
            drive.setControl(request.withVelocityX(v[0]).withVelocityY(v[1]).withTargetDirection(target));
        }).withName("Snap " + Math.round(target.getDegrees()));
    }

    /**
     * Drive with the sticks while the robot holds the field angle (a multiple of 90 degrees)
     * nearest its heading when the button is pressed: square to the field walls. The angle is
     * chosen once, at the press: worked out again every loop, it would flip between two angles
     * while the heading is near 45 degrees.
     */
    public static Command snapToNearestQuarterTurn(Drive drive, DoubleSupplier leftY, DoubleSupplier leftX) {
        var request = facingAngle(TargetDirectionPerspectiveValue.BlueAlliance);
        var target = new Rotation2d[] {Rotation2d.ZERO};
        return drive.startRun(() -> {
            target[0] = nearestQuarterTurn(drive.getHeading());
            Telemetry.log("Drive/Snap Target deg", target[0].getDegrees());
        }, () -> {
            double[] v = translation(leftY, leftX);
            drive.setControl(request.withVelocityX(v[0]).withVelocityY(v[1]).withTargetDirection(target[0]));
        }).withName("Snap Nearest 90");
    }

    /** The multiple of 90 degrees nearest {@code heading} (field frame). */
    public static Rotation2d nearestQuarterTurn(Rotation2d heading) {
        return Rotation2d.fromDegrees(90 * Math.round(heading.getDegrees() / 90));
    }

    /** Drive with the sticks while the robot front points at our hub. */
    public static Command faceHub(Drive drive, DoubleSupplier leftY, DoubleSupplier leftX) {
        var request = facingAngle(TargetDirectionPerspectiveValue.BlueAlliance);
        return drive.run(() -> {
            double[] v = translation(leftY, leftX);
            drive.setControl(request.withVelocityX(v[0]).withVelocityY(v[1])
                .withTargetDirection(angleToHub(drive.getPose(), Drive.isRedAlliance())));
        }).withName("Face Hub");
    }

    /**
     * Field angle from the robot to our hub. With no alliance known, the blue hub. (If the robot
     * is exactly on the hub center there is no direction, so keep the present heading.)
     */
    public static Rotation2d angleToHub(Pose2d pose, boolean red) {
        return FieldConstants.hub(red).minus(pose.getTranslation()).getAngle().orElse(pose.getRotation());
    }

    /** Wheels in an X: hard to push. Hold the button to keep it. */
    public static Command xLock(Drive drive) {
        var brake = new SwerveRequest.SwerveDriveBrake().withSteerRequestType(SteerRequestType.MotionMagicExpo);
        return drive.applyRequest(() -> brake).withName("X-Lock");
    }

    /** Set the gyro so the robot's present heading is "away from the driver". */
    public static Command resetHeading(Drive drive) {
        return Commands.runOnce(drive::seedFieldCentric, drive).ignoringDisable(true).withName("Reset Heading");
    }
}
