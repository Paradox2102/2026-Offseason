package frc.robot.subsystems.shooter;

import java.util.List;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.telemetry.Telemetry;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.constants.CanIds;
import frc.robot.health.HealthMonitor;
import frc.robot.lib.phoenix.PhoenixUtil;
import frc.robot.lib.sim.RollerSim;

/**
 * The path from the hopper to the flywheel: indexer roller (30), belt tunnels (31, 32), Scotty
 * rollers (45) and vertical roller (40). They run together: only when the shot is ready, or all
 * out to eject. Stopped, they brake, so a ball does not roll into the flywheel early.
 *
 * <p>Speeds are motor rps, the old code's values; inversions are the old code's too.
 */
public class Feeder extends SubsystemBase {
    /** One motor of the path. */
    private record Stage(String name, TalonFX talon, double feedRps, RollerSim sim) {}

    private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    private final StaticBrake m_brake = new StaticBrake();
    private final List<Stage> m_stages;
    private String m_state = "Stopped";

    public Feeder() {
        m_stages = List.of(
            stage("Indexer roller (30)", CanIds.indexerRoller, CanIds.k_canBus1, InvertedValue.Clockwise_Positive,
                ShooterConstants.kIndexerRollerRps, 0.125, 25, true),
            stage("Top belt tunnel (31)", CanIds.indexerConveyorLeader, CanIds.k_canBus1, InvertedValue.CounterClockwise_Positive,
                ShooterConstants.kBeltTunnelRps, 0.096, 18, false),
            stage("Bottom belt tunnel (32)", CanIds.indexerConveyorFollower, CanIds.k_canBus1, InvertedValue.Clockwise_Positive,
                ShooterConstants.kBeltTunnelRps, 0.096, 18, false),
            stage("Scotty rollers (45)", CanIds.goUp, CanIds.k_canBus3, InvertedValue.Clockwise_Positive,
                ShooterConstants.kScottyRps, 0.1195, 23, true),
            stage("Vertical roller (40)", CanIds.kicker, CanIds.k_canBus3, InvertedValue.CounterClockwise_Positive,
                ShooterConstants.kVerticalRollerRps, 0.118, 23, true));
    }

    private Stage stage(String name, int id, CANBus bus, InvertedValue inverted, double feedRps,
                        double kV, double supplyLimit, boolean x60) {
        var talon = new TalonFX(id, bus);
        var config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = inverted;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kV = kV; // V per motor rps (old code; belt tunnels: X44 12 V / 125 rps)
        config.Slot0.kP = 0.05;
        config.MotionMagic.MotionMagicAcceleration = 200; // motor rps per second: full speed in 0.15 s
        config.CurrentLimits.SupplyCurrentLimit = supplyLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        PhoenixUtil.applyConfig(name, () -> talon.getConfigurator().apply(config));
        HealthMonitor.register(talon);
        RollerSim sim = null;
        if (RobotBase.isSimulation()) {
            var orientation = inverted == InvertedValue.Clockwise_Positive
                ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            sim = new RollerSim(talon, x60 ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX44Foc(1), 1.0, 0.0002,
                orientation, x60 ? TalonFXSimState.MotorType.KrakenX60 : TalonFXSimState.MotorType.KrakenX44);
        }
        return new Stage(name, talon, feedRps, sim);
    }

    private void runAt(double fraction) {
        for (Stage s : m_stages) {
            s.talon().setControl(m_request.withVelocity(s.feedRps() * fraction));
        }
    }

    private void stop() {
        for (Stage s : m_stages) {
            s.talon().setControl(m_brake);
        }
        m_state = "Stopped";
    }

    /** Feed balls to the flywheel while {@code ready} is true; brake while it is false. */
    public Command feedWhen(java.util.function.BooleanSupplier ready) {
        return runEnd(() -> {
            if (ready.getAsBoolean()) {
                runAt(1.0);
                m_state = "Feeding";
            } else {
                stop();
                m_state = "Waiting";
            }
        }, this::stop).withName("Feed When Ready");
    }

    /** Everything out (a jam, or a ball we do not want). */
    public Command eject() {
        return runEnd(() -> {
            runAt(ShooterConstants.kEjectFraction);
            m_state = "Ejecting";
        }, this::stop).withName("Feeder Eject");
    }

    /** True while the balls are being fed to the flywheel. */
    public boolean isFeeding() {
        return m_state.equals("Feeding");
    }

    @Override
    public void periodic() {
        Telemetry.log("Shooter/Feeder/State", m_state);
        Telemetry.log("Shooter/Feeder/Vertical Roller rps", m_stages.get(4).talon().getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        for (Stage s : m_stages) {
            s.sim().update();
        }
    }
}
