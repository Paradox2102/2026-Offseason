package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.ShooterConstants;

public class HoodSubsystem extends SubsystemBase{

    private final TalonFX m_hoodMotor = new TalonFX(CANIDConstants.hood);
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    private final InterpolatingDoubleTreeMap m_hoodLerpTable = new InterpolatingDoubleTreeMap();

    public HoodSubsystem() {
        for (double[] shotAngle : ShooterConstants.m_hoodAngle){
            m_hoodLerpTable.put(shotAngle[0], shotAngle[1]);
        }
        m_hoodMotor.getConfigurator().apply(ShooterConstants.k_hoodConfig);
    }

    public Command pitchHood(double distanceToHub) {
        return Commands.run(() -> {
            Globals.setPosition(m_hoodMotor, m_request, m_hoodLerpTable.get(distanceToHub));
        }, this);
    }

    
}
