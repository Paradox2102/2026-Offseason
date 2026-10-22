package frc.robot.lib.phoenix;

import java.util.function.Supplier;

import org.wpilib.util.Alert.Level;

import com.ctre.phoenix6.StatusCode;

import frc.robot.lib.util.AlertPublisher;

/** Small helpers for Phoenix 6 devices. */
public final class PhoenixUtil {
    private static final int kTries = 5;

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
                return true;
            }
        }
        AlertPublisher.create("Phoenix", name + "/config",
            "Config failed on " + name + " (" + status.getName() + "): it runs with old settings",
            Level.HIGH).set(true);
        return false;
    }
}
