package frc.robot.constants;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Translation2d;

/**
 * Field facts. The origin is the blue alliance wall corner, +x toward the red wall, as in the
 * AprilTag layout and the BLine paths.
 */
public final class FieldConstants {
    private FieldConstants() {}

    /**
     * The AprilTag layout. Events use either the AndyMark or the welded field; tags differ by up
     * to 3.6 cm between the two. Change this one line for a welded-field event: vision, the vision
     * sim and the BLine alliance flip all read it.
     */
    public static final Field kField = Fields.FRC_2026_REBUILT_ANDY_MARK.loadField();
    public static final double kFieldLength = kField.getFieldLength();
    public static final double kFieldWidth = kField.getFieldWidth();

    /** The hub of our alliance. */
    public static Translation2d hub(boolean red) {
        return red ? k_redHub : k_blueHub;
    }

    public static final Translation2d k_blueHub = new Translation2d(4.62, 4.03);
    public static final Translation2d k_redHub = new Translation2d(11.92, 4.03);

    public static Translation2d k_blueOutpost = new Translation2d(1, 1);
    public static Translation2d k_blueDepot = new Translation2d(1, 7);

    public static Translation2d k_redOutpost = new Translation2d(15.6, 1);
    public static Translation2d k_redDepot = new Translation2d(15.6, 7);

    public static Translation2d k_fullPassRedDepot = new Translation2d(11.3, 5.5); // Need to implement this
    public static Translation2d k_fullPassRedOutpost = new Translation2d(11.3, 2.5);
    public static Translation2d k_fullPassBlueDepot = new Translation2d(5.3, 5.5);
    public static Translation2d k_fullPassBlueOutpost = new Translation2d(5.3, 2.5);

    public static final double k_redZoneTransition = 11.925;
    public static final double k_blueZoneTransition = 4.625;
    public static final double k_yCenterline = 4.0;
    public static final double k_depotIntermediatePoint = 5.940;
    public static final double k_outpostIntermediatePoint = 1.0;
    
    // public static final Rectangle2d k_blueTrenchLeft = new Rectangle2d(
    //     new Translation2d(2.5, 8),
    //     new Translation2d(6.6, 6.8)
    // );

    // public static final Rectangle2d k_blueTrenchRight = new Rectangle2d(
    //     new Translation2d(2.5, 1.3),
    //     new Translation2d(6.6, 0)
    // );

    // public static final Rectangle2d k_redTrenchLeft = new Rectangle2d(
    //     new Translation2d(14, 8),
    //     new Translation2d(10.2, 6.8)
    // );

    // public static final Rectangle2d k_redTrenchRight = new Rectangle2d(
    //     new Translation2d(14, 1.3),
    //     new Translation2d(10.2, 0)
    // );
}
