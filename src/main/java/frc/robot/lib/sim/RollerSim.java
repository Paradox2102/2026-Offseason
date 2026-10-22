package frc.robot.lib.sim;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.system.RobotController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

/**
 * Simulator only: a spinning mechanism (flywheel, roller, belt) on one Talon.
 *
 * <p>Each loop it reads the voltage the Talon puts out, spins a WPILib flywheel model and gives
 * the speed back to the Talon, so the Talon's own velocity loop runs as on the robot.
 */
public class RollerSim {
    private static final double kDt = 0.020;

    private final TalonFX m_talon;
    private final double m_ratio;
    private final FlywheelSim m_sim;

    /**
     * @param talon the motor's Talon
     * @param motor the motor model (for example {@code DCMotor.getKrakenX60Foc(1)})
     * @param ratio motor rotations per output rotation (above 1 = a reduction)
     * @param moi moment of inertia of the output, kg m^2
     * @param orientation must match the Talon's {@code Inverted} setting
     * @param type the Kraken model
     */
    public RollerSim(TalonFX talon, DCMotor motor, double ratio, double moi,
                     ChassisReference orientation, TalonFXSimState.MotorType type) {
        m_talon = talon;
        m_ratio = ratio;
        m_sim = new FlywheelSim(Models.flywheelFromPhysicalConstants(motor, moi, ratio), motor);
        var state = talon.getSimState();
        state.Orientation = orientation;
        state.setMotorType(type);
    }

    public void update() {
        var state = m_talon.getSimState();
        state.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_sim.setInputVoltage(state.getMotorVoltage());
        m_sim.update(kDt);
        double outputRps = m_sim.getAngularVelocity() / (2 * Math.PI);
        state.setRotorVelocity(outputRps * m_ratio);
        state.addRotorPosition(outputRps * m_ratio * kDt);
    }
}
