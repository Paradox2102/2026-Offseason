package frc.robot.calibration;

import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import org.wpilib.system.Timer;

import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.superstructure.Superstructure;

/**
 * Feeder tuning: the speed of each feed stage, and the fire rate.
 *
 * <p>Test pad: D-pad left/right pick a stage, up/down its speed (bumper held: bigger steps), X
 * (held) run that stage alone, RT shoot (a full hopper: the fire rate is counted from the flywheel's
 * speed dips), Y save. The speeds are used at once.
 */
public class FeederProcedure extends Procedure {
    private static final double kStep = 5; // motor rps
    private static final double kBigStep = 20;
    private final Calibration m_calibration;
    private final Feeder m_feeder;
    private final Flywheel m_flywheel;
    private final Superstructure m_superstructure;
    private final DipCounter m_dips = new DipCounter(0.95, 0.98);
    private int m_stage = 0;
    private boolean m_counting = false;

    FeederProcedure(Calibration calibration, Feeder feeder, Flywheel flywheel, Superstructure superstructure) {
        super("Feeder tuning");
        m_calibration = calibration;
        m_feeder = feeder;
        m_flywheel = flywheel;
        m_superstructure = superstructure;
        m_instructions = "D-pad left/right: stage; up/down: its speed (bumper: bigger steps). X (hold): run that stage alone."
            + " RT (hold): shoot a full hopper, the fire rate counts the flywheel dips. Y: save";
    }

    @Override
    void bind(CommandGamepad pad, Trigger active) {
        pad.dpadRight().and(active).onTrue(Commands.runOnce(() -> pickStage(1)));
        pad.dpadLeft().and(active).onTrue(Commands.runOnce(() -> pickStage(-1)));
        pad.dpadUp().and(active).onTrue(Commands.runOnce(() -> jog(pad.rightBumper().getAsBoolean() ? kBigStep : kStep)));
        pad.dpadDown().and(active).onTrue(Commands.runOnce(() -> jog(pad.rightBumper().getAsBoolean() ? -kBigStep : -kStep)));
        pad.faceLeft().and(active).whileTrue(m_feeder.runStage(() -> m_stage));
        pad.rightTrigger().and(active).whileTrue(fireRateTest());
    }

    /** Shoot normally and count the balls from the flywheel's dips. */
    public org.wpilib.command2.Command fireRateTest() {
        return m_superstructure.shoot()
            .beforeStarting(() -> {
                m_dips.reset();
                m_counting = true;
            })
            .finallyDo(() -> m_counting = false)
            .withName("Calibration Fire Rate");
    }

    public void pickStage(int step) {
        int n = m_feeder.stageCount();
        m_stage = (m_stage + step + n) % n;
    }

    /** Change the picked stage's feed speed by this much, motor rps; used at once. */
    public void jog(double rps) {
        double[] speeds = m_feeder.feedSpeeds();
        speeds[m_stage] = Math.max(0, speeds[m_stage] + rps);
        m_feeder.setFeedSpeeds(speeds);
        m_calibration.data().feedSpeeds = speeds;
    }

    public int dips() {
        return m_dips.count();
    }

    @Override
    void periodic() {
        if (m_counting) {
            m_dips.update(Timer.getTimestamp(), m_flywheel.getSpeed(), m_flywheel.getTarget());
        }
        m_step = m_counting ? "Counting the fire rate" : "Stage " + (m_stage + 1) + " of " + m_feeder.stageCount();
        m_readings[0] = String.format("%s: feed speed %.1f rps", m_feeder.stageName(m_stage), m_feeder.feedSpeeds()[m_stage]);
        m_readings[1] = String.format("Now %.1f rps, %.1f A", m_feeder.stageSpeed(m_stage), m_feeder.stageCurrent(m_stage));
        m_readings[2] = String.format("Fire rate: %d balls, %.1f per second", m_dips.count(), m_dips.rate());
        m_readings[3] = "";
    }

    /** The turret aims at the hub, as in a match. */
    @Override
    boolean holdTurret() {
        return false;
    }
}
