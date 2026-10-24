package frc.robot.subsystems.intake;

import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.system.RobotController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

/**
 * Simulator only: the two rack sides as two "elevators" with hard stops, with gravity along the
 * rack's 20 degree slope. The sides are not joined (the real frame flexes).
 *
 * <p>The stops make the push into the out stop work in the sim. The right side's out stop is 6 mm
 * past its constant (IntakeConstants.kOutStopRight), as a measured value that is a little off: it
 * stalls just past the stop and is fixed with no alert.
 */
class IntakeSim {
    private static final double kDt = 0.020;
    private static final double kSideMass = IntakeConstants.kRackSideMass;
    private static final double kGearing = IntakeConstants.kRackGearRatio;
    private static final double kPinionRadius = IntakeConstants.kRackPinionRadius;

    private final TalonFX m_leftTalon;
    private final TalonFX m_rightTalon;
    /** Where the sim's out stops are, meters: the left at its constant, the right 6 mm past. */
    static final double kLeftStop = IntakeConstants.kOutStopLeft;
    static final double kRightStop = IntakeConstants.kOutStopRight + 0.006;
    private final ElevatorSim m_left = side(kLeftStop);
    private final ElevatorSim m_right = side(kRightStop);
    /** Skipped teeth: how far the motor reading is off the real rack position, meters. */
    private double m_leftSkip = 0;
    private double m_rightSkip = 0;
    /** Something in the way of a side: it cannot go past this, meters (NaN = nothing). */
    private double m_leftBlock = Double.NaN;
    private double m_rightBlock = Double.NaN;
    /** Talon 22 does not answer: no power to its side, and its reading stays where it was. */
    private boolean m_rightDead = false;

    IntakeSim(TalonFX left, TalonFX right) {
        m_leftTalon = left;
        m_rightTalon = right;
        if ("out".equalsIgnoreCase(System.getenv("SIM_RACK_START"))) {
            // As on the robot: the rack slid out while the robot was off, and the Talons read 0 there.
            for (var side : new ElevatorSim[] {m_left, m_right}) {
                side.setState(side == m_left ? kLeftStop : kRightStop, 0);
            }
            m_leftSkip = -kLeftStop;
            m_rightSkip = -kRightStop;
            System.out.println("SIM rack starts out (SIM_RACK_START=out): the Talons read 0 at the out stops");
        }
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
        step(m_leftTalon.getSimState(), m_left, m_leftSkip, m_leftBlock, false);
        step(m_rightTalon.getSimState(), m_right, m_rightSkip, m_rightBlock, m_rightDead);
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

    void blockRight(double position) {
        m_rightBlock = position;
    }

    void killRight() {
        m_rightDead = true;
    }

    /** One side skips teeth: its reading moves, the rack does not. */
    void skipTeeth(boolean left, double meters) {
        if (left) {
            m_leftSkip += meters;
        } else {
            m_rightSkip += meters;
        }
    }

    private static void step(TalonFXSimState talon, ElevatorSim side, double skip, double block, boolean dead) {
        talon.setSupplyVoltage(RobotController.getBatteryVoltage());
        // Gravity down the 20 degree slope pushes the rack out: the same as this much more
        // voltage on the motor (at low speed, where the back voltage is small). A dead Talon
        // gives no voltage (its motor is shorted, as in brake: gravity alone moves the side).
        side.setInputVoltage((dead ? 0 : talon.getMotorVoltage()) + IntakeConstants.kGravityVolts);
        side.update(kDt);
        if (!Double.isNaN(block) && side.getPosition() > block) {
            side.setState(block, 0); // stopped by the obstacle: no speed past it
        }
        if (dead) {
            return; // nothing reaches the robot code: the last reading stays
        }
        talon.setRawRotorPosition((side.getPosition() + skip) * IntakeConstants.kRotationsPerMeter);
        talon.setRotorVelocity(side.getVelocity() * IntakeConstants.kRotationsPerMeter);
    }
}
