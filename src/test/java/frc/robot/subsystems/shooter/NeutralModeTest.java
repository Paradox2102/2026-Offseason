package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.signals.NeutralModeValue;

/** User, 2026-10-25: disabled, the turret and the hood must turn easily by hand. */
class NeutralModeTest {
    @Test
    void theTurretCoastsWhenItIsNeutral() {
        assertEquals(NeutralModeValue.Coast, Turret.config().MotorOutput.NeutralMode);
    }

    @Test
    void theHoodCoastsWhenItIsNeutral() {
        assertEquals(NeutralModeValue.Coast, Hood.config(ShooterConstants.kHoodRatio, 0).MotorOutput.NeutralMode);
    }
}
