package frc.robot.calibration;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import frc.robot.lib.util.AlertPublisher;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeRack;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.superstructure.Superstructure;

/**
 * The calibration tool: procedures that measure what only the robot can give (shot table, steer,
 * turret, intake and hood zero, feed speeds), and the file that keeps the results
 * ({@link CalibrationStore}). The values in the file are used over the constants from boot on.
 *
 * <p>Elastic tab "Calibration": pick the procedure, read its step, instructions and readings, type
 * values, Capture and Save. The test gamepad (port 2) works only in Utility mode (the 2027 name of
 * Test mode) and only for the procedure picked. Y saves (not in SysId, which uses Y).
 */
public class Calibration {
    private static CalibrationStore s_store;
    private static CalibrationData s_data;
    private static String s_loadProblem = null;

    private final List<Procedure> m_procedures = new ArrayList<>();
    private final Selectable<Procedure> m_chooser = new Selectable<>();
    private final Alert m_fileAlert = AlertPublisher.create("Health", "calibration/file", "", Level.MEDIUM);
    private final SysIdProcedure m_sysId;
    /** The angle the turret holds while a procedure that does not aim is picked, rotations. */
    private double m_turretHoldRot = 0;
    private Procedure m_selected;
    private String m_message = "";

    /**
     * The measured values, loaded from the file the first time (at boot, before the subsystems are
     * built). A missing file: none (the constants). A file that cannot be read: none, and an alert.
     */
    public static synchronized CalibrationData get() {
        if (s_data == null) {
            s_store = new CalibrationStore(CalibrationStore.defaultPath());
            try {
                s_data = s_store.load();
                System.out.println("Calibration: " + (s_store.exists() ? "loaded " + s_store.path() : "no file at "
                    + s_store.path() + ", constants only"));
                String lines = s_data.javaLines();
                if (!lines.isEmpty()) {
                    System.out.print(lines);
                }
            } catch (IOException | RuntimeException e) {
                s_data = new CalibrationData();
                s_loadProblem = "Calibration file " + s_store.path() + " not read (" + e.getMessage() + "): constants in use";
                System.out.println(s_loadProblem);
            }
        }
        return s_data;
    }

    public Calibration(CommandGamepad pad, Drive drive, Turret turret, Hood hood, Flywheel flywheel, Feeder feeder,
                       IntakeRack rack, Superstructure superstructure) {
        add(new ShotTableProcedure(this, superstructure, flywheel, hood));
        add(new SteerZeroProcedure(this, drive));
        add(new TurretZeroProcedure(this, turret));
        add(new IntakeZeroProcedure(this, rack));
        add(new HoodZeroProcedure(this, hood));
        add(new FeederProcedure(this, feeder, flywheel, superstructure));
        m_sysId = add(new SysIdProcedure(drive));
        m_selected = m_procedures.get(0);
        m_chooser.onChange(p -> m_selected = p);
        Tunables.publish("Calibration/Procedure", m_chooser);

        Trigger utility = new Trigger(RobotState::isUtility);
        // In Utility mode the turret's default command still aims at the hub. Hold it still for the
        // procedures that do not shoot. isUtility() is true while disabled too, and a command
        // scheduled while disabled is dropped, so the hold starts at the enable.
        new Trigger(RobotState::isUtilityEnabled).and(() -> m_selected.holdTurret())
            .whileTrue(turret.track(() -> m_turretHoldRot).beforeStarting(() -> m_turretHoldRot = turret.getPosition())
                .withName("Calibration Turret Hold"));
        for (Procedure p : m_procedures) {
            Trigger active = utility.and(() -> m_selected == p);
            p.bind(pad, active);
            if (p.saveWithY()) {
                pad.faceUp().and(active).onTrue(Commands.runOnce(this::save).ignoringDisable(true));
            }
        }
        Tunables.publish("Calibration/Capture", Commands.runOnce(this::capture).ignoringDisable(true).withName("Capture"));
        Tunables.publish("Calibration/Save", Commands.runOnce(this::save).ignoringDisable(true).withName("Save"));
        get();
        m_fileAlert.setText(s_loadProblem == null ? "" : s_loadProblem);
        m_fileAlert.set(s_loadProblem != null);
    }

    private <T extends Procedure> T add(T procedure) {
        if (m_procedures.isEmpty()) {
            m_chooser.addDefault(procedure.name(), procedure);
        } else {
            m_chooser.add(procedure.name(), procedure);
        }
        m_procedures.add(procedure);
        return procedure;
    }

    /** The values to change and save. */
    public CalibrationData data() {
        return get();
    }

    /** The dashboard's Capture button: the picked procedure's main action. */
    public void capture() {
        if (!RobotState.isUtility()) {
            m_message = "Capture works only in Utility mode";
            return;
        }
        m_selected.capture();
    }

    /** Write the file (a backup of the old one first) and print the values. */
    public boolean save() {
        if (!RobotState.isUtility()) {
            m_message = "Save works only in Utility mode";
            return false;
        }
        get();
        try {
            var backup = s_store.save(s_data);
            m_message = "Saved " + s_store.path() + (backup == null ? "" : " (old file kept as " + backup.getFileName() + ")");
            System.out.println("Calibration: " + m_message + ". Values, to put in the code:");
            System.out.print(s_data.javaLines());
            return true;
        } catch (IOException | RuntimeException e) {
            m_message = "NOT saved: " + e.getMessage();
            System.out.println("Calibration: " + m_message);
            return false;
        }
    }

    /** Sim test: pick a procedure without the dashboard. */
    public void select(String name) {
        for (var p : m_procedures) {
            if (p.name().equals(name)) {
                m_selected = p;
            }
        }
    }

    /** Sim test: the procedure of this class. */
    public <T extends Procedure> T procedure(Class<T> type) {
        return m_procedures.stream().filter(type::isInstance).map(type::cast).findFirst().orElseThrow();
    }

    /** Call every loop. */
    public void periodic() {
        m_selected.periodic();
        Telemetry.log("Calibration/Utility Mode", RobotState.isUtility());
        Telemetry.log("Calibration/Step", RobotState.isUtility() ? m_selected.step() : "Utility mode only (Test mode)");
        Telemetry.log("Calibration/Instructions", m_selected.instructions());
        for (int i = 0; i < 4; i++) {
            Telemetry.log("Calibration/Reading " + (i + 1), m_selected.reading(i));
        }
        Telemetry.log("Calibration/Message", m_message);
        Telemetry.log("Drive/SysId Routine", m_sysId.routine().name());
    }
}
