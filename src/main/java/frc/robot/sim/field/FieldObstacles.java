package frc.robot.sim.field;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.math.geometry.Pose2d;

/**
 * Simulator only: the field structures our robot cannot drive through, and the push that moves it
 * back out of them. The CTRE sim moves the robot wherever its wheels take it; this stops it at the
 * hub, the trench side walls, the tower uprights and the field walls, and at a trench arm when the
 * robot is too tall to pass under it (intake not out, or hood up).
 *
 * <p>Measured from AdvantageScope's 2026 field model, blue side; the red side is the same turned
 * 180 degrees about the field center. Each structure is a box on the floor plan (a robot is never
 * high enough to go over one). No hardware: unit tested.
 */
public final class FieldObstacles {
    private FieldObstacles() {}

    /** AdvantageScope's field model size, meters (the frame of the numbers below). */
    public static final double kLength = 16.541;
    public static final double kWidth = 8.069;
    /** Underside of the trench arms, meters (22.25 in). */
    public static final double kTrenchClearance = 0.565;

    /** A box on the floor plan, field frame. */
    public record Box(String name, double minX, double minY, double maxX, double maxY) {
        Box mirrored(String newName) {
            return new Box(newName, kLength - maxX, kWidth - maxY, kLength - minX, kWidth - minY);
        }
    }

    /** The result of one push: where the robot ends, and what it hit (null if nothing). */
    public record Contact(Pose2d pose, String obstacle) {}

    private static final List<Box> kSolid = new ArrayList<>();
    private static final List<Box> kTrenchArms = new ArrayList<>();

    private static void both(List<Box> list, Box blue, String redName) {
        list.add(blue);
        list.add(blue.mirrored(redName));
    }

    static {
        // Hub body: its four panels.
        both(kSolid, new Box("blue hub", 4.029, 3.438, 5.223, 4.631), "red hub");
        // Each trench: a lane under the arm, along the field wall, then a solid wall (risers and
        // skirts) between the lane and the bump.
        both(kSolid, new Box("blue trench wall (low y)", 4.029, 1.279, 5.222, 1.583), "red trench wall (high y)");
        both(kSolid, new Box("blue trench wall (high y)", 4.029, 6.486, 5.222, 6.790), "red trench wall (low y)");
        both(kTrenchArms, new Box("blue trench arm (low y)", 4.435, 0, 4.816, 1.279), "red trench arm (high y)");
        both(kTrenchArms, new Box("blue trench arm (high y)", 4.549, 6.790, 4.702, kWidth), "red trench arm (low y)");
        // Tower: two uprights; the robot fits under the support tubes (0.89 m) between them and
        // the wall.
        both(kSolid, new Box("blue tower upright", 1.018, 3.298, 1.107, 3.336), "red tower upright");
        both(kSolid, new Box("blue tower upright", 1.018, 4.155, 1.107, 4.193), "red tower upright");
        // Field walls, as thick boxes outside the carpet.
        kSolid.add(new Box("wall", -1, -1, 0, kWidth + 1));
        kSolid.add(new Box("wall", kLength, -1, kLength + 1, kWidth + 1));
        kSolid.add(new Box("wall", -1, -1, kLength + 1, 0));
        kSolid.add(new Box("wall", -1, kWidth, kLength + 1, kWidth + 1));
    }

    /**
     * Push the robot out of anything it overlaps.
     *
     * @param pose robot pose on the floor plan
     * @param rackOut intake travel out, meters
     * @param rackStroke full intake travel, meters
     * @param hoodDeg hood angle above its lower stop, degrees
     * @param floorZ height of the floor under the robot (the bumps), meters
     */
    public static Contact resolve(Pose2d pose, double rackOut, double rackStroke, double hoodDeg, double floorZ) {
        boolean tooTall = floorZ + RobotOutline.top(rackOut, rackStroke, hoodDeg) > kTrenchClearance;
        String hit = null;
        double x = pose.getX();
        double y = pose.getY();
        double cos = pose.getRotation().getCos();
        double sin = pose.getRotation().getSin();
        // A few passes: a push out of one box can push into its neighbor (a wall and the hub).
        for (int pass = 0; pass < 3; pass++) {
            boolean moved = false;
            for (var parts : robotParts(rackOut)) {
                for (var box : obstacles(tooTall)) {
                    double[] push = overlap(x, y, cos, sin, parts, box);
                    if (push != null) {
                        x += push[0];
                        y += push[1];
                        hit = box.name();
                        moved = true;
                    }
                }
            }
            if (!moved) {
                break;
            }
        }
        return new Contact(hit == null ? pose : new Pose2d(x, y, pose.getRotation()), hit);
    }

    /**
     * How far the top of the robot is below a trench arm, meters, while any part of it is under
     * one; infinity when it is under none. Negative = too tall.
     */
    public static double trenchHeadroom(Pose2d pose, double rackOut, double rackStroke, double hoodDeg, double floorZ) {
        double cos = pose.getRotation().getCos();
        double sin = pose.getRotation().getSin();
        for (var parts : robotParts(rackOut)) {
            for (var arm : kTrenchArms) {
                if (overlap(pose.getX(), pose.getY(), cos, sin, parts, arm) != null) {
                    return kTrenchClearance - floorZ - RobotOutline.top(rackOut, rackStroke, hoodDeg);
                }
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    private static List<Box> obstacles(boolean tooTallForTrench) {
        if (!tooTallForTrench) {
            return kSolid;
        }
        var all = new ArrayList<>(kSolid);
        all.addAll(kTrenchArms);
        return all;
    }

    /** Robot rectangles in the robot frame, {minX, maxX, halfWidth}: the body, and the intake when out. */
    private static List<double[]> robotParts(double rackOut) {
        var parts = new ArrayList<double[]>(2);
        parts.add(new double[] {RobotOutline.kBackX, RobotOutline.kFrontX, RobotOutline.kHalfWidth});
        if (RobotOutline.intakeOut(rackOut)) {
            parts.add(new double[] {RobotOutline.kFrontX, RobotOutline.intakeFront(rackOut), RobotOutline.kIntakeHalfWidth});
        }
        return parts;
    }

    /**
     * Separating-axis test between a robot rectangle (turned with the robot) and a box. Returns
     * the smallest push {dx, dy} that moves the robot out, or null if they do not overlap.
     */
    private static double[] overlap(double x, double y, double cos, double sin, double[] part, Box box) {
        double centerLocal = (part[0] + part[1]) / 2;
        double a = (part[1] - part[0]) / 2; // half length along the robot x
        double b = part[2]; // half width along the robot y
        double cx = x + centerLocal * cos;
        double cy = y + centerLocal * sin;
        double bx = (box.minX() + box.maxX()) / 2;
        double by = (box.minY() + box.maxY()) / 2;
        double ex = (box.maxX() - box.minX()) / 2;
        double ey = (box.maxY() - box.minY()) / 2;
        double[][] axes = {{1, 0}, {0, 1}, {cos, sin}, {-sin, cos}};
        double best = Double.POSITIVE_INFINITY;
        double[] push = null;
        for (double[] n : axes) {
            double robotRadius = a * Math.abs(cos * n[0] + sin * n[1]) + b * Math.abs(-sin * n[0] + cos * n[1]);
            double boxRadius = ex * Math.abs(n[0]) + ey * Math.abs(n[1]);
            double distance = (bx - cx) * n[0] + (by - cy) * n[1];
            double depth = robotRadius + boxRadius - Math.abs(distance);
            if (depth <= 0) {
                return null;
            }
            if (depth < best) {
                best = depth;
                double away = -Math.signum(distance == 0 ? 1 : distance);
                push = new double[] {away * depth * n[0], away * depth * n[1]};
            }
        }
        return push;
    }
}
