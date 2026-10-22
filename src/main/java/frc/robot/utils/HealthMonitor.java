package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;

import org.photonvision.PhotonCamera;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.can.CANStatus;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import frc.robot.lib.util.AlertPublisher;

import frc.robot.Robot;

/**
 * Watches the robot's electrical health and reports a problem EARLY, in two places:
 * <ul>
 * <li>an Alert on the dashboard (Elastic: add an Alerts widget for the "Health" group), so the
 * pit crew sees it before the next match, and</li>
 * <li>a value in the hoot log next to the motor signals, so we can see WHEN it happened.</li>
 * </ul>
 *
 * <p>Ideas from FRC 6328's 2026 code: CAN bus error counters, debounced "device disconnected"
 * alerts, and a log of every command that starts and ends.
 *
 * <p>Every ParadoxTalon registers itself. Robot registers the drivetrain, and each vision camera
 * registers itself. Robot calls {@link #start()} once and {@link #periodic()} every loop.
 */
public final class HealthMonitor {
    private HealthMonitor() {}

    private static final String k_group = "Health";
    /** The five CAN buses on the SystemCore. */
    private static final CANPort[] k_buses = {
        CANPort.CAN_S0, CANPort.CAN_S1, CANPort.CAN_S2, CANPort.CAN_S3, CANPort.CAN_S4
    };
    /** The slower checks run 4 times a second. CTRE sends fault frames at about this rate. */
    private static final double k_checkPeriod = 0.25;
    /** A battery below this voltage while disabled needs a change before the match. */
    private static final double k_lowBatteryVolts = 12.3;
    /** Only real hardware can fail. In simulation, keep the alerts off. */
    private static final boolean k_showHardwareAlerts = Robot.isReal();

    private static final List<WatchedBus> s_buses = new ArrayList<>();
    private static final List<WatchedDevice> s_devices = new ArrayList<>();
    private static final List<WatchedCamera> s_cameras = new ArrayList<>();

    private static final Timer s_checkTimer = new Timer();
    private static double s_lastLoopTime = -1;
    private static boolean s_reportedError = false;

    private static boolean s_wasBrownedOut = false;
    private static int s_brownouts = 0;
    private static final Alert s_brownoutAlert = AlertPublisher.create(k_group, "brownout", "", Level.HIGH);
    private static final Debouncer s_lowBatteryDebounce = new Debouncer(1.0, DebounceType.RISING);
    private static final Alert s_lowBatteryAlert = AlertPublisher.create(k_group, "lowBattery", "", Level.MEDIUM);
    private static int s_commsLossAtStart = -1;
    private static final Alert s_commsLossAlert = AlertPublisher.create(k_group, "commsLoss", "", Level.MEDIUM);
    private static int s_rail3V3FaultsAtStart = -1;
    private static final Alert s_rail3V3Alert = AlertPublisher.create(k_group, "rail3V3", "", Level.HIGH);

    /** Call once from the Robot constructor. */
    public static void start() {
        for (CANPort port : k_buses) {
            s_buses.add(new WatchedBus(port));
        }

        // Log every command, so a match log shows what the robot was trying to do.
        CommandScheduler.getInstance().onCommandInitialize(
            command -> SignalLogger.writeString("Health/Commands/Started", command.getName()));
        CommandScheduler.getInstance().onCommandFinish(
            command -> SignalLogger.writeString("Health/Commands/Finished", command.getName()));
        CommandScheduler.getInstance().onCommandInterrupt(
            command -> SignalLogger.writeString("Health/Commands/Interrupted", command.getName()));

        s_checkTimer.start();
    }

    /** Watch a CTRE device: is it on the CAN bus, and did it reboot or brown out? */
    public static void register(ParentDevice device) {
        addDevice(device, describe(device));
    }

    /** Watch all the motors, CANcoders and the Pigeon in a swerve drivetrain. */
    public static void registerDrivetrain(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        String[] corner = {"FL", "FR", "BL", "BR"}; // the order TunerConstants creates the modules
        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = drivetrain.getModules();
        for (int i = 0; i < modules.length; i++) {
            String prefix = (i < corner.length ? corner[i] : "Module " + i) + " ";
            addDevice(modules[i].getDriveMotor(), prefix + "drive " + describe(modules[i].getDriveMotor()));
            addDevice(modules[i].getSteerMotor(), prefix + "steer " + describe(modules[i].getSteerMotor()));
            addDevice(modules[i].getEncoder(), prefix + "encoder " + describe(modules[i].getEncoder()));
        }
        Pigeon2 pigeon = drivetrain.getPigeon2();
        addDevice(pigeon, "Gyro " + describe(pigeon));
    }

    /** Watch a PhotonVision camera: is its coprocessor still sending data? */
    public static void registerCamera(PhotonCamera camera) {
        for (WatchedCamera watched : s_cameras) {
            if (watched.m_camera == camera) {
                return;
            }
        }
        s_cameras.add(new WatchedCamera(camera));
    }

    /** Each alert needs a unique name, so a second registration of the same device is skipped. */
    private static void addDevice(ParentDevice device, String name) {
        for (WatchedDevice watched : s_devices) {
            if (watched.m_device == device) {
                return;
            }
        }
        s_devices.add(new WatchedDevice(device, name));
    }

    /** Call every loop from Robot.robotPeriodic(). */
    public static void periodic() {
        // A health check must never stop the robot. Report a failure once and keep going.
        try {
            runChecks();
        } catch (RuntimeException e) {
            if (!s_reportedError) {
                s_reportedError = true;
                DriverStationErrors.reportError("HealthMonitor failed: " + e, e.getStackTrace());
            }
        }
    }

    private static void runChecks() {
        // Loop time: the scheduler should run every 20 ms. Longer loops make the robot feel laggy.
        double now = Timer.getTimestamp();
        if (s_lastLoopTime >= 0) {
            SignalLogger.writeDouble("Health/System/LoopTime", (now - s_lastLoopTime) * 1000.0, "ms");
        }
        s_lastLoopTime = now;

        // Brownout: the SystemCore turned the motors off because the battery voltage fell too low.
        // A brownout is short, so check it every loop.
        boolean brownedOut = RobotController.isBrownedOut();
        SignalLogger.writeBoolean("Health/System/BrownedOut", brownedOut);
        SignalLogger.writeDouble("Health/System/BatteryVoltage", RobotController.getBatteryVoltage(), "V");
        if (brownedOut && !s_wasBrownedOut) {
            s_brownouts++;
            s_brownoutAlert.setText("Brownout " + s_brownouts + " time(s) since code start: check the battery and current limits");
        }
        s_wasBrownedOut = brownedOut;
        s_brownoutAlert.set(k_showHardwareAlerts && s_brownouts > 0);

        if (s_checkTimer.advanceIfElapsed(k_checkPeriod)) {
            checkSystem();
            for (WatchedBus bus : s_buses) {
                bus.check();
            }
            for (WatchedDevice device : s_devices) {
                device.check();
            }
            for (WatchedCamera camera : s_cameras) {
                camera.check();
            }
        }
    }

    private static void checkSystem() {
        double volts = RobotController.getBatteryVoltage();
        boolean low = RobotState.isDisabled() && volts < k_lowBatteryVolts;
        s_lowBatteryAlert.setText(String.format("Battery is %.1f V: change it before the match", volts));
        s_lowBatteryAlert.set(k_showHardwareAlerts && s_lowBatteryDebounce.calculate(low));

        // The DS disables the robot each time it loses comms (radio, Ethernet cable, field network).
        int commsLoss = RobotController.getCommsDisableCount();
        if (s_commsLossAtStart < 0) {
            s_commsLossAtStart = commsLoss;
        }
        int lost = commsLoss - s_commsLossAtStart;
        SignalLogger.writeInteger("Health/System/CommsDisableCount", commsLoss);
        s_commsLossAlert.setText("Lost DS comms " + lost + " time(s) since code start: check the radio and its cables");
        s_commsLossAlert.set(k_showHardwareAlerts && lost > 0);

        // A 3.3 V rail fault means a short on a sensor port.
        int railFaults = RobotController.getFaultCount3V3();
        if (s_rail3V3FaultsAtStart < 0) {
            s_rail3V3FaultsAtStart = railFaults;
        }
        int newRailFaults = railFaults - s_rail3V3FaultsAtStart;
        s_rail3V3Alert.setText("3.3 V rail fault " + newRailFaults + " time(s): check the sensor wiring for a short");
        s_rail3V3Alert.set(k_showHardwareAlerts && newRailFaults > 0);

        SignalLogger.writeDouble("Health/System/CPUTemp", RobotController.getCPUTemp(), "C");
    }

    /** For example "TalonFX 20 on CAN_S4". */
    private static String describe(ParentDevice device) {
        // ParadoxTalon is a TalonFX, so call it that in the alert text.
        String type = device instanceof TalonFX ? "TalonFX" : device.getClass().getSimpleName();
        return type + " " + device.getDeviceID() + " on " + device.getNetwork().getName();
    }

    /** One CAN bus: error counters from the SystemCore's CAN controller. */
    private static class WatchedBus {
        private final CANPort m_port;
        private final String m_key;
        private int m_busOffAtStart = -1;
        private int m_txFullAtStart = -1;
        private int m_lastReceiveErrors;
        private int m_lastTransmitErrors;
        // Keep the alert on for 2 s after the last error, so a short burst is still visible.
        private final Debouncer m_errorDebounce = new Debouncer(2.0, DebounceType.FALLING);
        private final Alert m_busOffAlert;
        private final Alert m_txFullAlert;
        private final Alert m_errorAlert;

        WatchedBus(CANPort port) {
            m_port = port;
            m_key = "Health/CAN/" + port.name() + "/";
            m_busOffAlert = AlertPublisher.create(k_group, port.name() + "/busOff", "", Level.HIGH);
            m_txFullAlert = AlertPublisher.create(k_group, port.name() + "/txFull", "", Level.MEDIUM);
            m_errorAlert = AlertPublisher.create(k_group, port.name() + "/errors", port.name() + " has CAN errors now: check its wiring and termination", Level.MEDIUM);
        }

        void check() {
            CANStatus status = RobotController.getCANStatus(m_port);
            SignalLogger.writeDouble(m_key + "Utilization", status.percentBusUtilization);
            SignalLogger.writeInteger(m_key + "BusOffCount", status.busOffCount);
            SignalLogger.writeInteger(m_key + "TxFullCount", status.txFullCount);
            SignalLogger.writeInteger(m_key + "ReceiveErrorCount", status.receiveErrorCount);
            SignalLogger.writeInteger(m_key + "TransmitErrorCount", status.transmitErrorCount);

            if (m_busOffAtStart < 0) {
                m_busOffAtStart = status.busOffCount;
                m_txFullAtStart = status.txFullCount;
                m_lastReceiveErrors = status.receiveErrorCount;
                m_lastTransmitErrors = status.transmitErrorCount;
            }

            // Bus-off: so many errors that the CAN controller stopped sending. Every device on
            // this bus goes quiet until it recovers.
            int busOffs = status.busOffCount - m_busOffAtStart;
            m_busOffAlert.setText(m_port.name() + " went bus-off " + busOffs + " time(s) since code start: check its wiring and termination");
            m_busOffAlert.set(k_showHardwareAlerts && busOffs > 0);

            // TX full: the SystemCore had more to send than the bus could carry.
            int txFulls = status.txFullCount - m_txFullAtStart;
            m_txFullAlert.setText(m_port.name() + " send buffer was full " + txFulls + " time(s) since code start: the bus is too busy or blocked");
            m_txFullAlert.set(k_showHardwareAlerts && txFulls > 0);

            // Error counters going up means bad frames on the wire right now (a loose
            // connector, a missing terminator, a damaged wire).
            boolean newErrors = status.receiveErrorCount > m_lastReceiveErrors
                || status.transmitErrorCount > m_lastTransmitErrors;
            m_lastReceiveErrors = status.receiveErrorCount;
            m_lastTransmitErrors = status.transmitErrorCount;
            m_errorAlert.set(k_showHardwareAlerts && m_errorDebounce.calculate(newErrors));
        }
    }

    /** One CTRE device on CAN. */
    private static class WatchedDevice {
        private final ParentDevice m_device;
        // Only alert after 0.5 s of silence, so one late frame does not cause a false alarm.
        private final Debouncer m_missingDebounce = new Debouncer(0.5, DebounceType.RISING);
        private final Alert m_disconnectedAlert;
        private final List<StickyFault> m_faults = new ArrayList<>();

        WatchedDevice(ParentDevice device, String name) {
            m_device = device;
            m_disconnectedAlert = AlertPublisher.create(k_group, name + "/disconnected", name + " is not on CAN", Level.HIGH);
            if (device instanceof TalonFX talon) {
                m_faults.add(new StickyFault(talon.getStickyFault_BootDuringEnable(false), name + " rebooted while enabled", Level.HIGH));
                m_faults.add(new StickyFault(talon.getStickyFault_BridgeBrownout(false), name + " motor output browned out", Level.MEDIUM));
            } else if (device instanceof CANcoder encoder) {
                m_faults.add(new StickyFault(encoder.getStickyFault_BootDuringEnable(false), name + " rebooted while enabled", Level.HIGH));
            } else if (device instanceof Pigeon2 pigeon) {
                m_faults.add(new StickyFault(pigeon.getStickyFault_BootDuringEnable(false), name + " rebooted while enabled", Level.HIGH));
            }
        }

        void check() {
            boolean missing = !m_device.isConnected();
            m_disconnectedAlert.set(k_showHardwareAlerts && m_missingDebounce.calculate(missing));
            for (StickyFault fault : m_faults) {
                fault.check();
            }
        }
    }

    /**
     * A sticky fault stays on in the device until someone clears it (Tuner X: "Clear Faults").
     * So only a fault that turns on AFTER the code starts is news. A fault that was already on
     * gets a LOW alert, so someone can clear it.
     */
    private static class StickyFault {
        private final StatusSignal<Boolean> m_signal;
        private final Alert m_newAlert;
        private final Alert m_oldAlert;
        private Boolean m_onAtStart = null;

        StickyFault(StatusSignal<Boolean> signal, String text, Level level) {
            m_signal = signal;
            m_newAlert = AlertPublisher.create(k_group, text, text, level);
            m_oldAlert = AlertPublisher.create(k_group, text + "/old", text + " (before code start: clear it in Tuner X)", Level.LOW);
        }

        void check() {
            m_signal.refresh(); // takes the latest value the device sent; it does not wait
            if (!m_signal.getStatus().isOK()) {
                return; // no data yet, or the device is missing (that has its own alert)
            }
            boolean on = m_signal.getValue();
            if (m_onAtStart == null) {
                m_onAtStart = on;
            }
            if (m_onAtStart && !on) {
                m_onAtStart = false; // someone cleared it, so watch for a new one
            }
            m_oldAlert.set(k_showHardwareAlerts && m_onAtStart && on);
            m_newAlert.set(k_showHardwareAlerts && !m_onAtStart && on);
        }
    }

    /** One PhotonVision camera. */
    private static class WatchedCamera {
        private final PhotonCamera m_camera;
        private final Debouncer m_missingDebounce = new Debouncer(1.0, DebounceType.RISING);
        private final Alert m_alert;

        WatchedCamera(PhotonCamera camera) {
            m_camera = camera;
            m_alert = AlertPublisher.create(k_group, "camera/" + camera.getName(), "Camera " + camera.getName() + " is not sending data: check its Pi, power and Ethernet", Level.HIGH);
        }

        void check() {
            boolean missing = !m_camera.isConnected();
            m_alert.set(k_showHardwareAlerts && m_missingDebounce.calculate(missing));
        }
    }
}
