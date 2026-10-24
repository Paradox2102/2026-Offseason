package frc.robot.subsystems.vision;

import java.util.List;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;

/**
 * Where each camera is on the robot.
 *
 * <p>Source: the CAD (Onshape "G0000 - Main", 2026-10-19). The numbers below are the origin of
 * each camera part; the code moves each camera {@link #kLensOffsetIn} along its lens axis, to
 * halfway between that origin and the lens front, close to where the lens takes its picture
 * from. x and y are from the robot center (+x = intake side, +y = left), z is from the bottom of
 * the wheels (the floor: in the CAD the wheel bottoms are at z = 0). The angles are the camera
 * part's lens axis (its -Y axis), as WPILib {@code Rotation3d(roll, pitch, yaw)}: roll about x,
 * then pitch about y, then yaw about z, all in the robot frame. A NEGATIVE pitch tilts the camera
 * UP.
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

    /**
     * One camera: its PhotonVision name, where it sits on the robot, and how much less to trust it
     * than the others (1 = the same; 2 = twice the standard deviation, for a camera that a log
     * shows to be noisier).
     *
     * <p>A factor that is not above 0 (a typo: 0 would make every frame of that camera exact) stops
     * the robot code at boot, on purpose.
     */
    public record CameraConfig(String name, Transform3d robotToCamera, double stdDevFactor) {
        public CameraConfig {
            if (!(stdDevFactor > 0)) { // NaN fails too
                throw new IllegalArgumentException("stdDevFactor must be > 0 for camera " + name);
            }
        }
    }

    /**
     * From the camera part origin along the lens axis, inches: half of the 0.98 in from the origin
     * to the lens front (the same for all four cameras in the CAD).
     */
    public static final double kLensOffsetIn = 0.49;

    /** A camera at a CAD part origin (inches, degrees), moved {@link #kLensOffsetIn} along its lens. */
    private static CameraConfig camera(String name, double xIn, double yIn, double zIn,
                                       double rollDeg, double pitchDeg, double yawDeg) {
        var rotation = new Rotation3d(Units.degreesToRadians(rollDeg), Units.degreesToRadians(pitchDeg),
            Units.degreesToRadians(yawDeg));
        // The camera looks along its own +x axis.
        var lens = new Translation3d(Units.inchesToMeters(kLensOffsetIn), 0, 0).rotateBy(rotation);
        var origin = new Translation3d(Units.inchesToMeters(xIn), Units.inchesToMeters(yIn), Units.inchesToMeters(zIn));
        return new CameraConfig(name, new Transform3d(origin.plus(lens), rotation), 1.0);
    }

    /** Named by where each camera sits (the PhotonVision names, the same as on main). */
    public static final List<CameraConfig> kCameras = List.of(
        camera("back_left", -11.53, +10.05, 8.69, -17.3, -30.0, -148.0),  // CAD cambrea 1 (moved 2026-10-19); looks back and right
        camera("back_right", -11.44, -9.09, 16.22, -2.1, -25.0, +148.0),  // CAD cambrea 2; looks back and left
        camera("side_right", -11.43, -12.73, 16.18, -11.1, -25.0, -65.0), // CAD cambrea 3; looks right
        camera("side_left", -0.80, +13.01, 9.13, -8.9, -30.0, +65.0));    // CAD cambrea 4; looks left

    /**
     * Have the camera angles above been checked on the robot? Until then vision does not set the
     * heading on the robot (the sim always lets it: its cameras are exact). A camera angle a few
     * degrees off would pull the robot's heading away from the gyro, and turn field-centric
     * driving and the turret aim with it. Set to true when the Vision tab shows robot roll, pitch
     * and z near 0 for every camera while the robot sits flat. While false, the heading comes from
     * the gyro, the auto start pose and the Reset Heading button.
     */
    public static final boolean kCamerasChecked = false;

    /** Simulated camera (the values the old code used; set them from a real calibration). */
    public static final int kSimWidth = 960;
    public static final int kSimHeight = 720;
    public static final double kSimDiagonalFovDeg = 100;
    public static final double kSimFps = 30;
    public static final double kSimLatencyMs = 35;
}
