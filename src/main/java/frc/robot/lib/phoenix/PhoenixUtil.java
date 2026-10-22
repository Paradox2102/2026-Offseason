package frc.robot.lib.phoenix;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import com.ctre.phoenix6.StatusCode;

import frc.robot.lib.util.AlertPublisher;

/** Small helpers for Phoenix 6 devices. */
public final class PhoenixUtil {
    private static final int kTries = 5;
    /** One alert per device name: making a second Alert with the same id throws. */
    private static final Map<String, Alert> s_alerts = new HashMap<>();

    private PhoenixUtil() {}

    /**
     * Apply a config, and try again if the device does not answer.
     *
     * <p>A config apply can fail when the bus is busy at startup or a device is still booting. If
     * nobody checks, the motor runs with its old (or factory) gains, inversion and current limits,
     * which is a silent and dangerous fault. So we try up to 5 times and raise an alert if it never
     * works.
     *
     * @param name shown in the alert, for example "Intake rack leader (21)"
     * @param apply for example {@code () -> motor.getConfigurator().apply(config)}
     * @return true if the config is on the device
     */
    public static boolean applyConfig(String name, Supplier<StatusCode> apply) {
        StatusCode status = StatusCode.OK;
        for (int i = 0; i < kTries; i++) {
            status = apply.get();
            if (status.isOK()) {
                var alert = s_alerts.get(name);
                if (alert != null) {
                    alert.set(false); // a later try (after a reboot) worked
                }
                return true;
            }
        }
        Alert alert = s_alerts.computeIfAbsent(name, n -> AlertPublisher.create("Health", "config/" + n, "", Level.HIGH));
        alert.setText("Config failed on " + name + " (" + status.getName() + "): it runs with old settings");
        alert.set(true);
        return false;
    }
}
