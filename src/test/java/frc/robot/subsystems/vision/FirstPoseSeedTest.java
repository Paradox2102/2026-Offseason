package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import frc.robot.subsystems.vision.FirstPoseSeed.Candidate;

class FirstPoseSeedTest {
    private static Candidate c(String camera, double x, double y, double deg, double time) {
        return new Candidate(camera, new Pose2d(x, y, Rotation2d.fromDegrees(deg)), time);
    }

    @Test
    void twoCamerasThatAgreeGiveTheirMean() {
        // Robot 2026-10-25 15:56: side_left and side_right, one tag each, both near (14.2, 4.1).
        var seed = FirstPoseSeed.find(List.of(c("side_left", 14.16, 4.27, -67, 10.00), c("side_right", 14.21, 3.97, -69, 10.05)));
        assertTrue(seed.isPresent());
        assertEquals(14.185, seed.get().getX(), 1e-9);
        assertEquals(4.12, seed.get().getY(), 1e-9);
        assertEquals(-68, seed.get().getRotation().getDegrees(), 1e-6);
    }

    @Test
    void oneCameraAloneIsNeverEnoughHoweverManyFrames() {
        // A flipped single-tag solution repeats frame after frame in the same camera.
        var seed = FirstPoseSeed.find(List.of(c("side_left", 14.2, 4.1, -67, 10.00), c("side_left", 14.2, 4.1, -67, 10.03),
            c("side_left", 14.2, 4.1, -67, 10.06)));
        assertTrue(seed.isEmpty());
    }

    @Test
    void camerasThatDisagreeGiveNothing() {
        assertTrue(FirstPoseSeed.find(List.of(c("side_left", 14.2, 4.1, -67, 10.0), c("side_right", 13.2, 4.1, -67, 10.0))).isEmpty(),
            "1 m apart");
        assertTrue(FirstPoseSeed.find(List.of(c("side_left", 14.2, 4.1, -67, 10.0), c("side_right", 14.2, 4.1, -40, 10.0))).isEmpty(),
            "27 deg apart");
    }

    @Test
    void framesTooFarApartInTimeDoNotCount() {
        assertTrue(FirstPoseSeed.find(List.of(c("side_left", 14.2, 4.1, -67, 10.0), c("side_right", 14.2, 4.1, -67, 10.5))).isEmpty());
    }

    @Test
    void anOutlierCameraIsLeftOut() {
        var seed = FirstPoseSeed.find(List.of(c("side_left", 14.2, 4.0, -67, 10.0), c("side_right", 14.2, 4.2, -67, 10.0),
            c("back_left", 11.0, 2.0, 110, 10.0)));
        assertTrue(seed.isPresent());
        assertEquals(4.1, seed.get().getY(), 1e-9);
    }

    @Test
    void theMeanHeadingWorksAcross180Degrees() {
        var seed = FirstPoseSeed.find(List.of(c("a", 5, 5, 179, 1.0), c("b", 5, 5, -177, 1.0)));
        assertTrue(seed.isPresent());
        assertEquals(-179, seed.get().getRotation().getDegrees(), 1e-6);
    }

    @Test
    void noCandidatesNoSeed() {
        assertTrue(FirstPoseSeed.find(List.of()).isEmpty());
    }
}
