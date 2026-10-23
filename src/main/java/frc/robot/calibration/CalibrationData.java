package frc.robot.calibration;

import java.util.ArrayList;
import java.util.List;

import frc.robot.calibration.ShotPoints.ShotPoint;

/**
 * The values measured on the robot. Each one is optional: NaN (or null for a list) = not measured,
 * use the constant.
 */
public class CalibrationData {
    /** Recorded shots, by distance. */
    public List<ShotPoint> shotPoints = new ArrayList<>();
    /** Swerve steer offsets FL, FR, BL, BR, rotations (null: TunerConstants). */
    public double[] steerOffsets = null;
    /** The turret reading at the jig angle, degrees. */
    public double turretTrimDeg = Double.NaN;
    /** Intake out stops, meters. */
    public double intakeOutStopLeft = Double.NaN;
    public double intakeOutStopRight = Double.NaN;
    /** Hood motor turns per hood turn. */
    public double hoodRatio = Double.NaN;
    /** The hood's angle above level at its lower stop, degrees. */
    public double hoodStopAboveLevelDeg = Double.NaN;
    /** Feed speeds per stage (indexer, top belt, bottom belt, Scotty, vertical roller), motor rps (null: constants). */
    public double[] feedSpeeds = null;

    /** The values as Java constant lines, to copy into the code later. */
    public String javaLines() {
        var out = new StringBuilder();
        if (!shotPoints.isEmpty()) {
            out.append("// Shot points {distance m, flywheel wheel rps, hood deg, time of flight s (NaN: not measured)}\n");
            for (var p : shotPoints) {
                out.append(String.format("{%.3f, %.2f, %.2f, %s},%n", p.distance(), p.flywheelRps(), p.hoodDeg(),
                    Double.isNaN(p.timeOfFlight()) ? "Double.NaN" : String.format("%.3f", p.timeOfFlight())));
            }
        }
        if (steerOffsets != null) {
            String[] names = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
            for (int i = 0; i < steerOffsets.length; i++) {
                out.append(String.format("k%sEncoderOffset = Rotations.of(%.5f);%n", names[i], steerOffsets[i]));
            }
        }
        line(out, "turret trim deg (Turret)", turretTrimDeg);
        line(out, "kOutStopLeft", intakeOutStopLeft);
        line(out, "kOutStopRight", intakeOutStopRight);
        line(out, "kHoodRatio", hoodRatio);
        if (!Double.isNaN(hoodStopAboveLevelDeg)) {
            out.append(String.format("kHoodStopAboveLevelRot = %.5f; // %.2f deg%n", hoodStopAboveLevelDeg / 360, hoodStopAboveLevelDeg));
        }
        if (feedSpeeds != null) {
            String[] names = {"kIndexerRollerRps", "kTopBeltRps", "kBottomBeltRps", "kScottyRps", "kVerticalRollerRps"};
            for (int i = 0; i < feedSpeeds.length && i < names.length; i++) {
                line(out, names[i], feedSpeeds[i]);
            }
        }
        return out.toString();
    }

    private static void line(StringBuilder out, String name, double value) {
        if (!Double.isNaN(value)) {
            out.append(String.format("%s = %.5f;%n", name, value));
        }
    }
}
