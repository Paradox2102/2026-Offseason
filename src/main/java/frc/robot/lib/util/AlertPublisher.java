package frc.robot.lib.util;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Creates Alerts and shows them in Elastic.
 *
 * <p>In WPILib 2027 an {@code Alert} goes to the SystemCore's own alert list, but not to
 * NetworkTables, so Elastic cannot see it. This class keeps every alert the robot makes and, each
 * loop, publishes the active ones in the format of Elastic's "Alerts" widget:
 * {@code /Telemetry/Alerts/<group>/errors}, {@code warnings} and {@code infos}.
 *
 * <p>Always make alerts with {@link #create}, never with {@code new Alert(...)}, or they will not
 * show on the dashboard. Alert ids must be unique in their group: a second alert with the same id
 * throws when the robot code starts.
 */
public final class AlertPublisher {
    private static final Map<String, List<Alert>> s_groups = new LinkedHashMap<>();

    private AlertPublisher() {}

    /** HIGH shows as an error, MEDIUM as a warning, LOW as an info. */
    public static synchronized Alert create(String group, String id, String text, Level level) {
        Alert alert = new Alert(group, id, text, level);
        s_groups.computeIfAbsent(group, g -> new ArrayList<>()).add(alert);
        return alert;
    }

    /** Text of the active alerts of one group and level. */
    public static synchronized String[] active(String group, Level level) {
        List<Alert> alerts = s_groups.getOrDefault(group, List.of());
        return alerts.stream()
            .filter(a -> a.get() && a.getLevel() == level)
            .map(Alert::getText)
            .toArray(String[]::new);
    }

    /** Call once per loop. */
    public static synchronized void publish() {
        for (String group : s_groups.keySet()) {
            var table = Telemetry.getTable("Alerts/" + group);
            table.setType("Alerts");
            table.log("errors", active(group, Level.HIGH));
            table.log("warnings", active(group, Level.MEDIUM));
            table.log("infos", active(group, Level.LOW));
        }
    }
}
