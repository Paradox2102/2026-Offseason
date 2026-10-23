package frc.robot.calibration;

import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

/**
 * Steer zero: every wheel lined up straight forward with a straight edge, then Capture. The swerve
 * is built at boot, so the new offsets are used after the next robot code restart.
 *
 * <p>Test pad: A capture, Y save.
 */
public class SteerZeroProcedure extends Procedure {
    private static final String[] kNames = {"FL", "FR", "BL", "BR"};
    private final Calibration m_calibration;
    private final Drive m_drive;
    private double[] m_captured = null;

    SteerZeroProcedure(Calibration calibration, Drive drive) {
        super("Steer zero");
        m_calibration = calibration;
        m_drive = drive;
        m_instructions = "Robot disabled first: line every wheel up straight forward with a straight edge, bevel gears"
            + " the way Tuner X had them. Then enable in Utility mode and press A (or Capture). Y: save."
            + " The new offsets are used after the next robot code restart";
    }

    @Override
    void bind(CommandGamepad pad, Trigger active) {
        pad.faceDown().and(active).onTrue(Commands.runOnce(this::capture).ignoringDisable(true));
    }

    /** The CANcoders' absolute readings now, rotations (FL, FR, BL, BR). */
    private double[] readings() {
        var modules = m_drive.getModules();
        double[] out = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            out[i] = modules[i].getEncoder().getAbsolutePosition().getValueAsDouble();
        }
        return out;
    }

    /** New offsets from the readings now and the offsets in use; stored, used after a restart. */
    @Override
    public void capture() {
        double[] reading = readings();
        double[] inUse = TunerConstants.encoderOffsets();
        m_captured = new double[reading.length];
        for (int i = 0; i < reading.length; i++) {
            m_captured[i] = SteerOffsets.newOffset(inUse[i], reading[i]);
        }
        m_calibration.data().steerOffsets = m_captured.clone();
    }

    /** The offsets of the last capture (null: none yet). */
    public double[] captured() {
        return m_captured == null ? null : m_captured.clone();
    }

    @Override
    void periodic() {
        double[] reading = readings();
        double[] inUse = TunerConstants.encoderOffsets();
        m_step = m_captured == null ? "Line the wheels up, then capture"
            : "Captured: save (Y), then restart the robot code to use them";
        var now = new StringBuilder("Reading deg:");
        var offsets = new StringBuilder("Offsets in use rot:");
        for (int i = 0; i < reading.length; i++) {
            now.append(String.format(" %s %.1f", kNames[i], reading[i] * 360));
            offsets.append(String.format(" %s %.4f", kNames[i], inUse[i]));
        }
        m_readings[0] = now.toString();
        m_readings[1] = offsets.toString();
        if (m_captured != null) {
            var captured = new StringBuilder("New offsets rot:");
            for (int i = 0; i < m_captured.length; i++) {
                captured.append(String.format(" %s %.4f", kNames[i], m_captured[i]));
            }
            m_readings[2] = captured.toString();
        } else {
            m_readings[2] = "";
        }
        m_readings[3] = "";
    }
}
