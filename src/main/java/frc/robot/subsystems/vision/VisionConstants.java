package frc.robot.subsystems.vision;

import java.util.List;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;

/**
 * Where each camera is on the robot.
 *
 * <p>Source: the CAD (Onshape "G0000 - Main", 2026-10-19), at the center of each lens front.
 * x and y are from the robot center (+x = intake side, +y = left), z is from the bottom of the
 * wheels. Angles are WPILib {@code Rotation3d(roll, pitch, yaw)}: roll about x, then pitch about y,
 * then yaw about z, all in the robot frame. A NEGATIVE pitch tilts the camera UP.
 *
 * <p>How to check on the robot:
 * <ul>
 *   <li>Roll: the CAD camera is a plain square, so it cannot show how the sensor is turned in it.
 *       Each PhotonVision stream must show a level horizon. A camera turned 90 or 180 degrees in
 *       its mount changes the roll by that much. "Vision/&lt;name&gt;/Robot Roll deg" and "Robot Pitch
 *       deg" should stay near 0 while the robot sits flat: if one camera reads a few degrees, its
 *       angles here are off by about that much.
 *   <li>Names: cover one lens at a time and see which PhotonVision camera goes dark.
 * </ul>
 */
public final class VisionConstants {
    private VisionConstants() {}

    /** One camera: its PhotonVision name and where it sits on the robot. */
    public record CameraConfig(String name, Transform3d robotToCamera) {}

    private static CameraConfig camera(String name, double xIn, double yIn, double zIn,
                                       double rollDeg, double pitchDeg, double yawDeg) {
        return new CameraConfig(name, new Transform3d(
            new Translation3d(Units.inchesToMeters(xIn), Units.inchesToMeters(yIn), Units.inchesToMeters(zIn)),
            new Rotation3d(Units.degreesToRadians(rollDeg), Units.degreesToRadians(pitchDeg), Units.degreesToRadians(yawDeg))));
    }

    /*
     * The two rear cameras cross: front_right sits at the back left and looks back and to the
     * right; front_left sits at the back right and looks back and to the left. (The PhotonVision
     * names were chosen when the shooter end was called the front.)
     */
    public static final List<CameraConfig> kCameras = List.of(
        camera("front_right", -12.25, +9.60, 9.18, -17.3, -30.0, -148.0), // CAD cambrea 1 (moved 2026-10-19)
        camera("front_left", -12.19, -8.62, 16.63, -2.1, -25.0, +148.0),  // CAD cambrea 2
        camera("side_left", -11.05, -13.54, 16.60, -11.1, -25.0, -65.0),  // CAD cambrea 3 (right side, looks right)
        camera("side_right", -0.44, +13.78, 9.62, -8.9, -30.0, +65.0));   // CAD cambrea 4; PhotonVision name not confirmed

    /** Simulated camera (the values the old code used; set them from a real calibration). */
    public static final int kSimWidth = 960;
    public static final int kSimHeight = 720;
    public static final double kSimDiagonalFovDeg = 100;
    public static final double kSimFps = 30;
    public static final double kSimLatencyMs = 35;
}
