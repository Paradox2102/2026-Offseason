package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.wpilib.math.util.Units;

class VisionConstantsTest {
    @Test
    void camerasUseTheNamesOfTheirPlaces() {
        // CAD cambrea 1 to 4, named by where they sit on the robot (+y = left), as on main.
        assertEquals(List.of("back_left", "back_right", "side_right", "side_left"),
            VisionConstants.kCameras.stream().map(VisionConstants.CameraConfig::name).toList());
    }

    @Test
    void camerasAreHalfwayBetweenThePartOriginAndTheLensFront() {
        // Inches: the average of the CAD part origin and the lens front, for each camera.
        double[][] expected = {
            {-11.89, 9.825, 8.935},
            {-11.815, -8.855, 16.425},
            {-11.24, -13.135, 16.39},
            {-0.62, 13.395, 9.375},
        };
        for (int i = 0; i < expected.length; i++) {
            var t = VisionConstants.kCameras.get(i).robotToCamera().getTranslation();
            String name = VisionConstants.kCameras.get(i).name();
            assertEquals(expected[i][0], Units.metersToInches(t.getX()), 0.03, name + " x");
            assertEquals(expected[i][1], Units.metersToInches(t.getY()), 0.03, name + " y");
            assertEquals(expected[i][2], Units.metersToInches(t.getZ()), 0.03, name + " z");
        }
    }
}
