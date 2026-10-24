package frc.robot.calibration;

import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;

/**
 * One calibration procedure: its step, its instructions and its readings for the dashboard, and
 * its test gamepad buttons. Each button calls a public method, so the sim test can do the same
 * without a gamepad.
 */
public abstract class Procedure {
    private final String m_name;
    protected String m_step = "";
    protected String m_instructions = "";
    protected final String[] m_readings = {"", "", "", ""};

    protected Procedure(String name) {
        m_name = name;
    }

    public String name() {
        return m_name;
    }

    public String step() {
        return m_step;
    }

    public String instructions() {
        return m_instructions;
    }

    public String reading(int i) {
        return m_readings[i];
    }

    /**
     * Bind the test gamepad.
     *
     * @param active Utility mode, and this procedure picked on the dashboard
     */
    abstract void bind(CommandGamepad pad, Trigger active);

    /** Each loop while picked: the step and the readings. */
    void periodic() {}

    /** The dashboard's Capture button (Utility mode only). */
    public void capture() {}

    /**
     * While this procedure is picked and the robot is enabled, Calibration holds the turret still
     * instead of letting it aim at the hub. False: the procedure aims or moves the turret itself.
     */
    boolean holdTurret() {
        return true;
    }

    /** Y saves the file (SysId uses Y for a test). */
    boolean saveWithY() {
        return true;
    }
}
