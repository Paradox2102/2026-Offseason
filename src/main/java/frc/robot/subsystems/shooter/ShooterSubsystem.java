package frc.robot.subsystems.shooter;

import javax.naming.ldap.SortResponseControl;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.Globals;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private final TalonFX k_shooterLeader = new TalonFX(CANIDConstants.shooterLeader);
    private final TalonFX k_shooterFollower = new TalonFX(CANIDConstants.shooterFollower);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final InterpolatingDoubleTreeMap m_shooterLerpTable = new InterpolatingDoubleTreeMap();

    private final double k_gearRatio = 0.5;

    private final FlywheelSim m_motorSimModel = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), IntakeConstants.k_intakeRollerMOI, k_gearRatio), DCMotor.getKrakenX60(1));

    public ShooterSubsystem() {
        k_shooterFollower.getConfigurator().apply(ShooterConstants.k_shooterConfigs);
        k_shooterLeader.getConfigurator().apply(ShooterConstants.k_shooterConfigs);

        for (double[] shotPower : ShooterConstants.m_shotSpeeds){
            m_shooterLerpTable.put(shotPower[0], shotPower[1]);
        
        if (Utils.isSimulation()){
        var talonFXSim = k_shooterLeader.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        }
    }

    public Command shoot(double distanceToHub) {
    return Commands.run(() -> {               
        Globals.setFollowerRPM(k_shooterLeader, k_shooterFollower, m_request, m_shooterLerpTable.get(distanceToHub),
        MotorAlignmentValue.Aligned);   
    }, this); }



        @Override
    public void simulationPeriodic() {
    var talonFXSim = k_shooterLeader.getSimState();
}
}
