package frc.robot.calibration;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.wpilib.framework.RobotBase;

import frc.robot.calibration.ShotPoints.ShotPoint;

/**
 * The calibration file: JSON, one value per measured thing (see {@link CalibrationData}). Unit
 * tested (a save and load round trip, the backup).
 *
 * <p>On the robot: /home/systemcore/calibration.json. In the sim: build/calibration.json. The
 * environment variable CALIBRATION_FILE picks another file (the sim scripts use their own, so a
 * file saved in a GUI sim does not change their results).
 *
 * <p>A save first copies the old file to calibration-backup/ next to it (with the time in its
 * name), then writes a temporary file and moves it over the old one, so a crash in the middle
 * never leaves half a file.
 */
public class CalibrationStore {
    private final Path m_path;

    public CalibrationStore(Path path) {
        m_path = path;
    }

    /** The file this robot uses. */
    public static Path defaultPath() {
        String env = System.getenv("CALIBRATION_FILE");
        if (env != null && !env.isBlank()) {
            return Path.of(env);
        }
        return RobotBase.isReal() ? Path.of("/home/systemcore/calibration.json") : Path.of("build", "calibration.json");
    }

    public Path path() {
        return m_path;
    }

    public boolean exists() {
        return Files.exists(m_path);
    }

    /** The values in the file; none if there is no file. */
    public CalibrationData load() throws IOException {
        if (!exists()) {
            return new CalibrationData();
        }
        return fromJson(Files.readString(m_path));
    }

    /**
     * Write the values. The old file is kept in calibration-backup/.
     *
     * @return the backup of the old file, or null if there was none
     */
    public Path save(CalibrationData data) throws IOException {
        Path dir = m_path.toAbsolutePath().getParent();
        Files.createDirectories(dir);
        Path backup = null;
        if (exists()) {
            Path backups = dir.resolve("calibration-backup");
            Files.createDirectories(backups);
            String time = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss-SSS"));
            backup = backups.resolve("calibration-" + time + ".json");
            Files.copy(m_path, backup, StandardCopyOption.REPLACE_EXISTING);
        }
        Path temp = dir.resolve(m_path.getFileName() + ".tmp");
        Files.writeString(temp, toJson(data));
        Files.move(temp, m_path, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
        return backup;
    }

    // ---- JSON: the few shapes this file needs (objects, arrays, numbers, null) ----

    static String toJson(CalibrationData data) {
        var out = new StringBuilder("{\n");
        out.append("  \"shotPoints\": [");
        for (int i = 0; i < data.shotPoints.size(); i++) {
            var p = data.shotPoints.get(i);
            out.append(i == 0 ? "\n" : ",\n").append(String.format(
                "    {\"distance\": %s, \"flywheelRps\": %s, \"hoodDeg\": %s, \"timeOfFlight\": %s}",
                number(p.distance()), number(p.flywheelRps()), number(p.hoodDeg()), number(p.timeOfFlight())));
        }
        out.append(data.shotPoints.isEmpty() ? "],\n" : "\n  ],\n");
        out.append("  \"steerOffsets\": ").append(array(data.steerOffsets)).append(",\n");
        out.append("  \"turretTrimDeg\": ").append(number(data.turretTrimDeg)).append(",\n");
        out.append("  \"intakeOutStopLeft\": ").append(number(data.intakeOutStopLeft)).append(",\n");
        out.append("  \"intakeOutStopRight\": ").append(number(data.intakeOutStopRight)).append(",\n");
        out.append("  \"hoodRatio\": ").append(number(data.hoodRatio)).append(",\n");
        out.append("  \"hoodStopAboveLevelDeg\": ").append(number(data.hoodStopAboveLevelDeg)).append(",\n");
        out.append("  \"feedSpeeds\": ").append(array(data.feedSpeeds)).append("\n");
        return out.append("}\n").toString();
    }

    /** NaN (not measured) is null in the file. */
    private static String number(double value) {
        return Double.isNaN(value) || Double.isInfinite(value) ? "null" : Double.toString(value);
    }

    private static String array(double[] values) {
        if (values == null) {
            return "null";
        }
        var out = new StringBuilder("[");
        for (int i = 0; i < values.length; i++) {
            out.append(i == 0 ? "" : ", ").append(number(values[i]));
        }
        return out.append("]").toString();
    }

    @SuppressWarnings("unchecked")
    static CalibrationData fromJson(String text) {
        var root = (Map<String, Object>) new Parser(text).value();
        var data = new CalibrationData();
        if (root.get("shotPoints") instanceof List<?> points) {
            var list = new ArrayList<ShotPoint>();
            for (Object o : points) {
                var p = (Map<String, Object>) o;
                list.add(new ShotPoint(num(p.get("distance")), num(p.get("flywheelRps")), num(p.get("hoodDeg")),
                    num(p.get("timeOfFlight"))));
            }
            data.shotPoints = list;
        }
        data.steerOffsets = nums(root.get("steerOffsets"));
        data.turretTrimDeg = num(root.get("turretTrimDeg"));
        data.intakeOutStopLeft = num(root.get("intakeOutStopLeft"));
        data.intakeOutStopRight = num(root.get("intakeOutStopRight"));
        data.hoodRatio = num(root.get("hoodRatio"));
        data.hoodStopAboveLevelDeg = num(root.get("hoodStopAboveLevelDeg"));
        data.feedSpeeds = nums(root.get("feedSpeeds"));
        return data;
    }

    private static double num(Object value) {
        return value instanceof Double d ? d : Double.NaN;
    }

    private static double[] nums(Object value) {
        if (!(value instanceof List<?> list)) {
            return null;
        }
        return list.stream().mapToDouble(CalibrationStore::num).toArray();
    }

    /** Reads JSON into maps, lists, doubles, strings, booleans and nulls. */
    private static final class Parser {
        private final String m_text;
        private int m_pos = 0;

        Parser(String text) {
            m_text = text;
        }

        Object value() {
            skipSpace();
            if (m_pos >= m_text.length()) {
                throw error("the file ends too soon");
            }
            char c = m_text.charAt(m_pos);
            if (c == '{') {
                return object();
            } else if (c == '[') {
                return list();
            } else if (c == '"') {
                return string();
            } else if (m_text.startsWith("null", m_pos)) {
                m_pos += 4;
                return null;
            } else if (m_text.startsWith("true", m_pos)) {
                m_pos += 4;
                return true;
            } else if (m_text.startsWith("false", m_pos)) {
                m_pos += 5;
                return false;
            }
            int start = m_pos;
            while (m_pos < m_text.length() && "+-0123456789.eE".indexOf(m_text.charAt(m_pos)) >= 0) {
                m_pos++;
            }
            try {
                return Double.parseDouble(m_text.substring(start, m_pos));
            } catch (NumberFormatException e) {
                throw error("not a value");
            }
        }

        private Map<String, Object> object() {
            var out = new LinkedHashMap<String, Object>();
            m_pos++; // {
            skipSpace();
            if (peek() == '}') {
                m_pos++;
                return out;
            }
            while (true) {
                skipSpace();
                String key = string();
                skipSpace();
                expect(':');
                out.put(key, value());
                skipSpace();
                if (peek() == ',') {
                    m_pos++;
                } else {
                    expect('}');
                    return out;
                }
            }
        }

        private List<Object> list() {
            var out = new ArrayList<Object>();
            m_pos++; // [
            skipSpace();
            if (peek() == ']') {
                m_pos++;
                return out;
            }
            while (true) {
                out.add(value());
                skipSpace();
                if (peek() == ',') {
                    m_pos++;
                } else {
                    expect(']');
                    return out;
                }
            }
        }

        /** A string with no escapes (the file has only plain key names). */
        private String string() {
            expect('"');
            int end = m_text.indexOf('"', m_pos);
            if (end < 0) {
                throw error("a string does not end");
            }
            String out = m_text.substring(m_pos, end);
            m_pos = end + 1;
            return out;
        }

        private char peek() {
            return m_pos < m_text.length() ? m_text.charAt(m_pos) : '\0';
        }

        private void expect(char c) {
            if (peek() != c) {
                throw error("'" + c + "' expected");
            }
            m_pos++;
        }

        private void skipSpace() {
            while (m_pos < m_text.length() && Character.isWhitespace(m_text.charAt(m_pos))) {
                m_pos++;
            }
        }

        private IllegalArgumentException error(String what) {
            return new IllegalArgumentException(what + " at character " + m_pos);
        }
    }
}
