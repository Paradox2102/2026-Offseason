package frc.robot.subsystems.intake;

import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.system.RobotController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

/**
 * Simulator only: the two rack sides as two "elevators" with hard stops, with gravity along the
 * rack's 20 degree slope.
 *
 * <p>The stops make the push into the out stop work in the sim. The right side's out stop is 6 mm
 * past the CAD stroke, to show that each side measures its own.
 */
class IntakeSim {
    private static final double kDt = 0.020;
    private static final double kSideMass = IntakeConstants.kRackSideMass;
    private static final double kGearing = IntakeConstants.kRackGearRatio;
    private static final double kPinionRadius = IntakeConstants.kRackPinionRadius;

    private final TalonFX m_leftTalon;
    private final TalonFX m_rightTalon;
    private final ElevatorSim m_left = side(IntakeConstants.kRackStroke);
    private final ElevatorSim m_right = side(IntakeConstants.kRackStroke + 0.006);
    /** Skipped teeth: how far the motor reading is off the real rack position, meters. */
    private double m_leftSkip = 0;
    private double m_rightSkip = 0;
    /** Something in the way of the left side: it cannot go past this, meters (NaN = nothing). */
    private double m_leftBlock = Double.NaN;

    IntakeSim(TalonFX left, TalonFX right) {
        m_leftTalon = left;
        m_rightTalon = right;
        setup(left.getSimState(), ChassisReference.CounterClockwise_Positive);
        setup(right.getSimState(), ChassisReference.Clockwise_Positive);
    }

    private static ElevatorSim side(double stroke) {
        return new ElevatorSim(DCMotor.getKrakenX44Foc(1), kGearing, kSideMass, kPinionRadius, 0.0, stroke, false, 0.0);
    }

    private static void setup(TalonFXSimState sim, ChassisReference orientation) {
        sim.Orientation = orientation;
        sim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }

    void update() {
        step(m_leftTalon.getSimState(), m_left, m_leftSkip, m_leftBlock);
        step(m_rightTalon.getSimState(), m_right, m_rightSkip, Double.NaN);
    }

    /** A hit pushes one side in (the motor is backdriven, so its reading follows). */
    void hit(boolean left, double meters) {
        var side = left ? m_left : m_right;
        side.setState(Math.max(0, side.getPosition() - meters), 0);
    }

    /** Put something in the way of the left side at this position (NaN: take it away). */
    void blockLeft(double position) {
        m_leftBlock = position;
    }

    /** One side skips teeth: its reading moves, the rack does not. */
    void skipTeeth(boolean left, double meters) {
        if (left) {
            m_leftSkip += meters;
        } else {
            m_rightSkip += meters;
        }
    }

    private static void step(TalonFXSimState talon, ElevatorSim side, double skip, double block) {
        talon.setSupplyVoltage(RobotController.getBatteryVoltage());
        // Gravity down the 20 degree slope pushes the rack out: the same as this much more
        // voltage on the motor (at low speed, where the back voltage is small).
        side.setInputVoltage(talon.getMotorVoltage() + IntakeConstants.kGravityVolts);
        side.update(kDt);
        if (!Double.isNaN(block) && side.getPosition() > block) {
            side.setState(block, 0); // stopped by the obstacle: no speed past it
        }
        talon.setRawRotorPosition((side.getPosition() + skip) * IntakeConstants.kRotationsPerMeter);
        talon.setRotorVelocity(side.getVelocity() * IntakeConstants.kRotationsPerMeter);
    }
}
