package frc.robot.calibration;

/**
 * Swerve steer offsets from a capture with every wheel pointing straight forward. No hardware:
 * unit tested.
 *
 * <p>A CANcoder adds its magnet offset to what it reads: absolute = raw + offset. With the wheel
 * straight forward the absolute reading must be 0, so the new offset is the old one minus what it
 * reads now, wrapped to half a turn each way.
 */
public final class SteerOffsets {
    private SteerOffsets() {}

    /**
     * @param oldOffset the offset in use (the one the CANcoder has now), rotations
     * @param absolute what the CANcoder reads now with the wheel straight forward, rotations
     * @return the offset that makes it read 0 there, rotations in [-0.5, 0.5)
     */
    public static double newOffset(double oldOffset, double absolute) {
        return wrap(oldOffset - absolute);
    }

    /** To [-0.5, 0.5) rotations. */
    public static double wrap(double rotations) {
        return rotations - Math.floor(rotations + 0.5);
    }
}
