package frc.robot.sim.field;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Simulator only: every ball of fuel on the field, and the ones in our hopper.
 *
 * <p>A small physics model, enough to see the game in AdvantageScope:
 *
 * <ul>
 *   <li>On the floor, a ball rolls with friction, rolls down the bump ramps, and bounces off the
 *       walls, the hub base, the robot and the other balls.
 *   <li>A ball that touches the intake while it collects goes into the hopper.
 *   <li>A shot ball flies with gravity and air drag. If it falls through the hub opening it
 *       scores and comes back out on the neutral-zone side of the hub; if not, it lands and rolls.
 * </ul>
 *
 * <p>The robot has no mass here: the balls never push it. No hardware: unit tested.
 */
public class FuelField {
    public static final double kRadius = 0.075;
    private static final double kMass = 0.23;
    private static final double kGravity = 9.81;
    /** Quadratic air drag, 1/m: 0.5 x air density x Cd 0.47 x area / mass. */
    private static final double kDrag = 0.5 * 1.2 * 0.47 * Math.PI * kRadius * kRadius / kMass;
    /** Foam on carpet: a rolling ball loses this much speed per second, m/s^2. */
    private static final double kRollingDecel = 1.0;
    private static final double kWallBounce = 0.3;
    private static final double kBallBounce = 0.3;
    private static final double kRobotBounce = 0.2;
    /** A falling ball bounces if it lands faster than this, m/s; slower, it stays on the floor. */
    private static final double kLandSpeed = 1.0;
    private static final double kLandBounce = 0.3;
    private static final double kLandFriction = 0.7;
    private static final int kSubsteps = 4;

    public static final int kHopperCapacity = 40;

    /** Hub (blue; red is turned 180 degrees about the field center). */
    public static final double kHubBlueX = 4.625;
    public static final double kHubY = 4.035;
    /** Below its panel tops (1.27 m) the hub is a 1.19 m square; above, the funnel is round enough. */
    public static final double kHubHalfSize = 0.597;
    private static final double kHubPanelTopZ = 1.27;
    public static final double kHubRadius = 0.6;
    public static final double kHubTopZ = 1.83;
    public static final double kHubOpeningRadius = 0.53;
    /**
     * A scored ball rolls down the ramp inside the hub and out of the gap above the rear panel
     * (0.77 m high, on the neutral-zone side).
     */
    private static final double kHubExitZ = 0.85;
    private static final double kHubExitSpeed = 1.5;


    /** What the robot does this step. Velocities are field-relative. */
    public record RobotInput(Pose2d pose, double vx, double vy, double omega, double rackOut, double rackStroke,
                             boolean rollersIn) {}

    private static final class Ball {
        private static int s_next = 0;
        final int id = s_next++;
        double x, y, z, vx, vy, vz;
        boolean inAir;
    }

    private final double m_fieldLength;
    private final double m_fieldWidth;
    private final List<Translation2d> m_start;
    private final int m_preload;
    private final List<Ball> m_balls = new ArrayList<>();
    private int m_hopper;
    private int m_pickedUp;
    private int m_shots;
    private int m_scoredBlue;
    private int m_scoredRed;
    private int m_exitCount;

    public FuelField(double fieldLength, double fieldWidth, List<Translation2d> start, int preload) {
        m_fieldLength = fieldLength;
        m_fieldWidth = fieldWidth;
        m_start = List.copyOf(start);
        m_preload = preload;
        reset();
    }

    /** Back to the start: staged balls on the floor, the preload in the hopper, counts at 0. */
    public void reset() {
        m_balls.clear();
        for (var p : m_start) {
            var b = new Ball();
            b.x = p.getX();
            b.y = p.getY();
            b.z = kRadius;
            m_balls.add(b);
        }
        m_hopper = m_preload;
        m_pickedUp = 0;
        m_shots = 0;
        m_scoredBlue = 0;
        m_scoredRed = 0;
    }

    /** Shoot one ball from the hopper, if it has one. Position and velocity in the field frame. */
    public boolean launch(Translation3d from, double vx, double vy, double vz) {
        if (m_hopper == 0) {
            return false;
        }
        m_hopper--;
        m_shots++;
        var b = new Ball();
        b.x = from.getX();
        b.y = from.getY();
        b.z = from.getZ();
        b.vx = vx;
        b.vy = vy;
        b.vz = vz;
        b.inAir = true;
        m_balls.add(b);
        return true;
    }

    /** Put a ball on the floor (tests, and any staged ball the sim adds). */
    public void addFloorBall(double x, double y) {
        var b = new Ball();
        b.x = x;
        b.y = y;
        b.z = BumpTerrain.height(x, y) + kRadius;
        m_balls.add(b);
    }

    public void step(double dt, RobotInput robot) {
        double h = dt / kSubsteps;
        for (int i = 0; i < kSubsteps; i++) {
            substep(h, robot);
        }
    }

    private void substep(double dt, RobotInput robot) {
        for (int i = m_balls.size() - 1; i >= 0; i--) {
            var b = m_balls.get(i);
            if (b.inAir) {
                fly(b, dt);
            } else {
                roll(b, dt);
            }
            hubBase(b);
            walls(b);
            if (!b.inAir && robot != null && robotContact(b, robot)) {
                m_balls.remove(i);
            }
        }
        collideBalls();
    }

    private void fly(Ball b, double dt) {
        double speed = Math.sqrt(b.vx * b.vx + b.vy * b.vy + b.vz * b.vz);
        b.vx -= kDrag * speed * b.vx * dt;
        b.vy -= kDrag * speed * b.vy * dt;
        b.vz -= (kGravity + kDrag * speed * b.vz) * dt;
        double oldZ = b.z;
        b.x += b.vx * dt;
        b.y += b.vy * dt;
        b.z += b.vz * dt;
        // Through the opening on the way down: scored.
        if (b.vz < 0 && oldZ >= kHubTopZ && b.z < kHubTopZ) {
            for (boolean red : new boolean[] {false, true}) {
                if (Math.hypot(b.x - hubX(red), b.y - hubY(red)) < kHubOpeningRadius) {
                    score(b, red);
                    return;
                }
            }
        }
        double floor = BumpTerrain.height(b.x, b.y) + kRadius;
        if (b.z <= floor) {
            b.z = floor;
            if (-b.vz > kLandSpeed) {
                b.vz = -b.vz * kLandBounce;
                b.vx *= kLandFriction;
                b.vy *= kLandFriction;
            } else {
                b.vz = 0;
                b.inAir = false;
            }
        }
    }

    private void score(Ball b, boolean red) {
        if (red) {
            m_scoredRed++;
        } else {
            m_scoredBlue++;
        }
        // Out on the neutral-zone side, spread along the hub face so the balls do not stack.
        double toNeutral = red ? -1 : 1;
        double spread = ((m_exitCount++ % 5) - 2) * 0.18;
        b.x = hubX(red) + toNeutral * (kHubHalfSize + kRadius + 0.01);
        b.y = hubY(red) + spread;
        b.z = kHubExitZ;
        b.vx = toNeutral * kHubExitSpeed;
        b.vy = 0;
        b.vz = 0;
        b.inAir = true;
    }

    private void roll(Ball b, double dt) {
        // Down the ramp: the slope part of gravity.
        double s = BumpTerrain.slopeX(b.x, b.y);
        b.vx -= kGravity * s / (1 + s * s) * dt;
        double speed = Math.hypot(b.vx, b.vy);
        if (speed > 0) {
            double slower = Math.max(0, speed - kRollingDecel * dt);
            b.vx *= slower / speed;
            b.vy *= slower / speed;
        }
        b.x += b.vx * dt;
        b.y += b.vy * dt;
        b.z = BumpTerrain.height(b.x, b.y) + kRadius;
    }

    private void walls(Ball b) {
        if (b.x < kRadius) {
            b.x = kRadius;
            b.vx = Math.abs(b.vx) * kWallBounce;
        } else if (b.x > m_fieldLength - kRadius) {
            b.x = m_fieldLength - kRadius;
            b.vx = -Math.abs(b.vx) * kWallBounce;
        }
        if (b.y < kRadius) {
            b.y = kRadius;
            b.vy = Math.abs(b.vy) * kWallBounce;
        } else if (b.y > m_fieldWidth - kRadius) {
            b.y = m_fieldWidth - kRadius;
            b.vy = -Math.abs(b.vy) * kWallBounce;
        }
    }

    /**
     * The hub is solid up to its top. Below the panel tops it is a square; above, the funnel (a
     * hexagon about 1.2 m across) is a cylinder of kHubRadius. Balls bounce off its sides.
     */
    private void hubBase(Ball b) {
        if (b.z > kHubTopZ) {
            return;
        }
        for (boolean red : new boolean[] {false, true}) {
            double dx = b.x - hubX(red);
            double dy = b.y - hubY(red);
            if (b.z < kHubPanelTopZ) {
                double reach = kHubHalfSize + kRadius;
                if (Math.abs(dx) < reach && Math.abs(dy) < reach) {
                    if (reach - Math.abs(dx) < reach - Math.abs(dy)) {
                        b.x = hubX(red) + Math.copySign(reach, dx);
                        b.vx = Math.copySign(Math.abs(b.vx) * kWallBounce, dx);
                    } else {
                        b.y = hubY(red) + Math.copySign(reach, dy);
                        b.vy = Math.copySign(Math.abs(b.vy) * kWallBounce, dy);
                    }
                }
                continue;
            }
            double d = Math.hypot(dx, dy);
            double reach = kHubRadius + kRadius;
            if (d < reach && d > 0) {
                double nx = dx / d;
                double ny = dy / d;
                b.x = hubX(red) + nx * reach;
                b.y = hubY(red) + ny * reach;
                double into = b.vx * nx + b.vy * ny;
                if (into < 0) {
                    b.vx -= (1 + kWallBounce) * into * nx;
                    b.vy -= (1 + kWallBounce) * into * ny;
                }
            }
        }
    }

    /**
     * A floor ball against the robot. Returns true if the intake took it. Otherwise the bumpers,
     * or the intake when it does not collect, push it out with the robot's speed at that point.
     */
    private boolean robotContact(Ball b, RobotInput robot) {
        var pose = robot.pose();
        double cos = pose.getRotation().getCos();
        double sin = pose.getRotation().getSin();
        double rx = b.x - pose.getX();
        double ry = b.y - pose.getY();
        // Ball in the robot frame.
        double lx = rx * cos + ry * sin;
        double ly = -rx * sin + ry * cos;

        double intakeFront = RobotOutline.intakeFront(robot.rackOut());
        boolean intakeOut = RobotOutline.intakeOut(robot.rackOut());
        boolean collecting = robot.rollersIn() && robot.rackOut() > robot.rackStroke() / 2 && m_hopper < kHopperCapacity;
        if (intakeOut && lx > RobotOutline.kFrontX - kRadius && lx < intakeFront + kRadius
                && Math.abs(ly) < RobotOutline.kIntakeHalfWidth + kRadius) {
            if (collecting) {
                m_hopper++;
                m_pickedUp++;
                return true;
            }
            // Out the front or the side: the bumper behind the intake is solid too.
            push(b, robot, lx, ly, cos, sin, RobotOutline.kBackX, intakeFront, RobotOutline.kIntakeHalfWidth);
            return false;
        }
        if (lx > RobotOutline.kBackX - kRadius && lx < RobotOutline.kFrontX + kRadius
                && Math.abs(ly) < RobotOutline.kHalfWidth + kRadius) {
            push(b, robot, lx, ly, cos, sin, RobotOutline.kBackX, RobotOutline.kFrontX, RobotOutline.kHalfWidth);
        }
        return false;
    }

    /** Push a ball out of the box [back, front] x [-halfWidth, halfWidth] (robot frame). */
    private void push(Ball b, RobotInput robot, double lx, double ly, double cos, double sin,
                      double back, double front, double halfWidth) {
        double toFront = front + kRadius - lx;
        double toBack = lx - (back - kRadius);
        double toSide = halfWidth + kRadius - Math.abs(ly);
        double nx;
        double ny;
        if (toFront <= toBack && toFront <= toSide) {
            lx = front + kRadius;
            nx = 1;
            ny = 0;
        } else if (toBack <= toSide) {
            lx = back - kRadius;
            nx = -1;
            ny = 0;
        } else {
            ly = Math.copySign(halfWidth + kRadius, ly);
            nx = 0;
            ny = Math.signum(ly);
        }
        var pose = robot.pose();
        b.x = pose.getX() + lx * cos - ly * sin;
        b.y = pose.getY() + lx * sin + ly * cos;
        double fnx = nx * cos - ny * sin;
        double fny = nx * sin + ny * cos;
        // The robot's speed at the contact point: its velocity plus omega x r.
        double px = b.x - pose.getX();
        double py = b.y - pose.getY();
        double rvx = robot.vx() - robot.omega() * py;
        double rvy = robot.vy() + robot.omega() * px;
        double relative = (b.vx - rvx) * fnx + (b.vy - rvy) * fny;
        if (relative < 0) {
            b.vx -= (1 + kRobotBounce) * relative * fnx;
            b.vy -= (1 + kRobotBounce) * relative * fny;
        }
    }

    /** Floor balls against each other, with a grid so each ball checks only its neighbors. */
    private void collideBalls() {
        double cell = 2 * kRadius;
        int cols = (int) Math.ceil(m_fieldLength / cell) + 1;
        var grid = new java.util.HashMap<Long, List<Ball>>();
        for (var b : m_balls) {
            if (!b.inAir) {
                long key = (long) Math.floor(b.x / cell) * cols * 4 + (long) Math.floor(b.y / cell);
                grid.computeIfAbsent(key, k -> new ArrayList<>(4)).add(b);
            }
        }
        double minDist = 2 * kRadius;
        for (var b : m_balls) {
            if (b.inAir) {
                continue;
            }
            long cx = (long) Math.floor(b.x / cell);
            long cy = (long) Math.floor(b.y / cell);
            for (long ix = cx - 1; ix <= cx + 1; ix++) {
                for (long iy = cy - 1; iy <= cy + 1; iy++) {
                    var others = grid.get(ix * cols * 4 + iy);
                    if (others == null) {
                        continue;
                    }
                    for (var o : others) {
                        if (o.id <= b.id) { // each pair once
                            continue;
                        }
                        double dx = o.x - b.x;
                        double dy = o.y - b.y;
                        double d2 = dx * dx + dy * dy;
                        if (d2 >= minDist * minDist || d2 == 0) {
                            continue;
                        }
                        double d = Math.sqrt(d2);
                        double nx = dx / d;
                        double ny = dy / d;
                        double overlap = (minDist - d) / 2;
                        b.x -= nx * overlap;
                        b.y -= ny * overlap;
                        o.x += nx * overlap;
                        o.y += ny * overlap;
                        double relative = (o.vx - b.vx) * nx + (o.vy - b.vy) * ny;
                        if (relative < 0) {
                            double j = -(1 + kBallBounce) * relative / 2;
                            b.vx -= j * nx;
                            b.vy -= j * ny;
                            o.vx += j * nx;
                            o.vy += j * ny;
                        }
                    }
                }
            }
        }
    }

    private double hubX(boolean red) {
        return red ? m_fieldLength - kHubBlueX : kHubBlueX;
    }

    private double hubY(boolean red) {
        return red ? m_fieldWidth - kHubY : kHubY;
    }

    /** Ball centers, for AdvantageScope. */
    public Translation3d[] positions() {
        var out = new Translation3d[m_balls.size()];
        for (int i = 0; i < out.length; i++) {
            var b = m_balls.get(i);
            out[i] = new Translation3d(b.x, b.y, b.z);
        }
        return out;
    }

    public int ballCount() {
        return m_balls.size();
    }

    public int airCount() {
        return (int) m_balls.stream().filter(b -> b.inAir).count();
    }

    public int hopper() {
        return m_hopper;
    }

    public int pickedUp() {
        return m_pickedUp;
    }

    public int shots() {
        return m_shots;
    }

    public int scored(boolean red) {
        return red ? m_scoredRed : m_scoredBlue;
    }
}
