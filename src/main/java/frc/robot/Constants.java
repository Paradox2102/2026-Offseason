package frc.robot;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.lights.LightSubsystem;

/**
 * What is left of the old constants. Each mechanism now has its own constants class next to it
 * (DriveConstants, IntakeConstants, ShooterConstants, VisionConstants); CAN IDs are in
 * constants/CanIds and field facts in constants/FieldConstants.
 */
public class Constants {

    public static class Subsystems {
        public static final Drive m_drive;
        public static final LightSubsystem m_lightSubsystem = new LightSubsystem();
        static {
            m_drive = TunerConstants.createDrivetrain();
        }
    }

    public static class LightConstants {
        public static final int k_lightPort = 0; // may be different on actual robot
        public static final int k_lightAmount = 100;

        public static LinearVelocity k_disabledVelocity = MetersPerSecond.of(0.2);
        public static Distance k_ledSpacing = Meters.of(1 / 120.0);
    }
}
