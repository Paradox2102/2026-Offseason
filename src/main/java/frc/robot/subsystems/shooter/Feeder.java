package frc.robot.subsystems.shooter;

import java.util.List;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

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
import frc.robot.lib.util.AlertPublisher;
import frc.robot.subsystems.shooter.FeedLogic.Mode;
import frc.robot.subsystems.shooter.FeedLogic.Out;

/**
 * The path from the hopper to the flywheel: indexer roller (30), belt tunnels (31, 32), Scotty
 * rollers (45) and vertical roller (40, the last one, into the wheel). They run together: only
 * when the shot is ready, or all out to eject. Stopped, they brake, so a ball does not roll into
 * the flywheel early; the vertical roller runs on for a moment first, so it does not hold a ball
 * against the spinning wheel. At the end of a shot they then all run back for a moment, so no ball
 * is left at the turret entrance. A stage that jams makes them all run back for a moment
 * ({@link FeedLogic}).
 *
 * <p>Speeds are motor rps: the CAEL robot's duty cycles as speeds (ShooterConstants). Inversions
 * are the old code's.
 */
public class Feeder extends SubsystemBase {
    /** One motor of the path. */
    private record Stage(String name, TalonFX talon, double feedRps, double statorLimit, RollerSim sim) {}

    private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    private final StaticBrake m_brake = new StaticBrake();
    private final List<Stage> m_stages;
    private final FeedLogic m_logic;
    private final Alert m_jamAlert = AlertPublisher.create("Health", "shooter/feeder/jams", "", Level.MEDIUM);
    private Mode m_mode = Mode.STOP;
    private String m_state = "Stopped";

    public Feeder() {
        double x60 = ShooterConstants.kFeedStatorLimitX60;
        double x44 = ShooterConstants.kFeedStatorLimitX44;
        m_stages = List.of(
            stage("Indexer roller (30)", CanIds.indexerRoller, CanIds.k_canBus1, InvertedValue.Clockwise_Positive,
                ShooterConstants.kIndexerRollerRps, ShooterConstants.kIndexerRollerKv, 25, x60, true),
            stage("Top belt tunnel (31)", CanIds.indexerConveyorLeader, CanIds.k_canBus1, InvertedValue.CounterClockwise_Positive,
                ShooterConstants.kTopBeltRps, ShooterConstants.kBeltTunnelKv, 18, x44, false),
            stage("Bottom belt tunnel (32)", CanIds.indexerConveyorFollower, CanIds.k_canBus1, InvertedValue.Clockwise_Positive,
                ShooterConstants.kBottomBeltRps, ShooterConstants.kBeltTunnelKv, 18, x44, false),
            stage("Scotty rollers (45)", CanIds.goUp, CanIds.k_canBus3, InvertedValue.Clockwise_Positive,
                ShooterConstants.kScottyRps, ShooterConstants.kScottyKv, 23, x60, true),
            stage("Vertical roller (40)", CanIds.kicker, CanIds.k_canBus3, InvertedValue.CounterClockwise_Positive,
                ShooterConstants.kVerticalRollerRps, ShooterConstants.kVerticalRollerKv, 23, x60, true));
        m_logic = new FeedLogic(m_stages.size(), ShooterConstants.kFeedRunOnTime, ShooterConstants.kFeedClearTime,
            ShooterConstants.kFeedJamTime,
            ShooterConstants.kFeedUnjamTime, ShooterConstants.kFeedJamAlertCount, ShooterConstants.kFeedJamWindow);
    }

    private Stage stage(String name, int id, CANBus bus, InvertedValue inverted, double feedRps,
                        double kV, double supplyLimit, double statorLimit, boolean x60) {
        var talon = new TalonFX(id, bus);
        var config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = inverted;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kV = kV;
        config.Slot0.kP = 0.05;
        config.MotionMagic.MotionMagicAcceleration = ShooterConstants.kFeedAcceleration;
        // A stalled stage under only a supply limit still draws ~100 A in the motor (X60).
        config.CurrentLimits.StatorCurrentLimit = statorLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
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
        return new Stage(name, talon, feedRps, statorLimit, sim);
    }

    /** Ask {@link FeedLogic} what each stage does now, and do it. */
    private void apply() {
        double now = Timer.getTimestamp();
        boolean[] stalled = new boolean[m_stages.size()];
        for (int i = 0; i < stalled.length; i++) {
            Stage s = m_stages.get(i);
            stalled[i] = Math.abs(s.talon().getVelocity().getValueAsDouble()) < ShooterConstants.kFeedJamSpeedFraction * s.feedRps()
                && s.talon().getStatorCurrent().getValueAsDouble() > ShooterConstants.kFeedJamCurrentFraction * s.statorLimit();
        }
        Out[] out = m_logic.update(now, m_mode, stalled);
        for (int i = 0; i < out.length; i++) {
            Stage s = m_stages.get(i);
            switch (out[i]) {
                case FEED -> s.talon().setControl(m_request.withVelocity(s.feedRps()));
                case REVERSE -> s.talon().setControl(m_request.withVelocity(s.feedRps() * ShooterConstants.kEjectFraction));
                case BRAKE -> s.talon().setControl(m_brake);
            }
        }
        if (m_mode == Mode.FEED) {
            m_state = m_logic.isUnjamming(now) ? "Unjamming" : "Feeding";
        } else if (m_mode == Mode.EJECT) {
            m_state = "Ejecting";
        } else if (m_logic.isClearing(now)) {
            m_state = "Clearing";
        } else if (m_state.equals("Clearing")) {
            m_state = "Stopped";
        }
    }

    private void set(Mode mode, String state) {
        m_mode = mode;
        m_state = state;
        apply();
    }

    /**
     * Feed balls to the flywheel while {@code ready} is true; pause while it is false. When the
     * command ends (the shot is over) the feed path clears: everything runs back for a moment.
     */
    public Command feedWhen(java.util.function.BooleanSupplier ready) {
        return runEnd(() -> {
            if (ready.getAsBoolean()) {
                set(Mode.FEED, "Feeding");
            } else {
                set(Mode.STOP, "Waiting");
            }
        }, () -> set(Mode.END, "Stopped")).withName("Feed When Ready");
    }

    /** Everything out (a jam, or a ball we do not want). */
    public Command eject() {
        return runEnd(() -> set(Mode.EJECT, "Ejecting"), () -> set(Mode.STOP, "Stopped")).withName("Feeder Eject");
    }

    /** For test output: jams in the last few seconds. */
    public int recentJams() {
        return m_logic.recentJams(Timer.getTimestamp());
    }

    /** True while every stage runs back at the end of a shot. */
    public boolean isClearing() {
        return m_logic.isClearing(Timer.getTimestamp());
    }

    /** The vertical roller's speed (the last stage, into the wheel), motor rps. */
    public double verticalRollerSpeed() {
        return m_stages.get(m_stages.size() - 1).talon().getVelocity().getValueAsDouble();
    }

    /** True while the balls are being fed to the flywheel (not while unjamming or clearing). */
    public boolean isFeeding() {
        return m_state.equals("Feeding");
    }

    @Override
    public void periodic() {
        double now = Timer.getTimestamp();
        // No command runs: finish the vertical roller's run-on and the clear. Disabled, the Talons
        // are neutral anyway: send nothing (at code start they may not be up yet).
        if ((m_mode == Mode.STOP || m_mode == Mode.END) && RobotState.isEnabled()) {
            apply();
        }
        int jams = m_logic.recentJams(now);
        int stage = m_logic.jammedStage();
        if (stage >= 0) {
            m_jamAlert.setText("Feeder jams again and again (last at " + m_stages.get(stage).name()
                + "): check for a stuck ball or a broken belt");
        }
        m_jamAlert.set(m_logic.jamAlert(now));
        Telemetry.log("Shooter/Feeder/State", m_state);
        Telemetry.log("Shooter/Feeder/Recent Jams", jams);
        Telemetry.log("Shooter/Feeder/Last Jam Stage", stage >= 0 ? m_stages.get(stage).name() : "");
        Telemetry.log("Shooter/Feeder/Vertical Roller rps", m_stages.get(4).talon().getVelocity().getValueAsDouble());
        for (Stage s : m_stages) {
            String prefix = "Shooter/Feeder/" + s.name() + "/";
            Telemetry.log(prefix + "Speed rps", s.talon().getVelocity().getValueAsDouble());
            Telemetry.log(prefix + "Current A", s.talon().getStatorCurrent().getValueAsDouble());
        }
    }

    @Override
    public void simulationPeriodic() {
        for (Stage s : m_stages) {
            s.sim().update();
        }
    }
}
