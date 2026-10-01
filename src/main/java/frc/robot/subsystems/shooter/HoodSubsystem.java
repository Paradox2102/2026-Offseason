package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Subsystems;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;

public class HoodSubsystem extends SubsystemBase{
    private final CANBus k_CANBus = CANBus.systemcore(1);

    private final TalonFX m_hoodMotor = new TalonFX(CANIDConstants.hood, k_CANBus);
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    private final InterpolatingDoubleTreeMap m_hoodLerpTable = new InterpolatingDoubleTreeMap();

    private final double k_gearRatio = 0.2; // what was I doing with this? this needs to be a models smth
    // The hood's actual angle is the motor's location times the gearRatio (actually divided because it's geared down)
    private final SingleJointedArmSim m_motorSimModel = new SingleJointedArmSim(DCMotor.getKrakenX44(1),
    k_gearRatio, ShooterConstants.k_hoodMOI, ShooterConstants.k_hoodArmLength, Math.toRadians(0), Math.toRadians(42), true, 0);
    
    public final Trigger m_isOnTarget = new Trigger(() -> (Math.abs(m_request.Position - m_hoodLerpTable.get(Subsystems.m_swerveSubsystem.distanceToTarget())) < 0.5));

    public HoodSubsystem() {
        for (double[] shotAngle : ShooterConstants.k_scoreHoodAngle){
            m_hoodLerpTable.put(shotAngle[0], shotAngle[1]);
        }
        m_hoodMotor.getConfigurator().apply(ShooterConstants.k_hoodConfig);

        if (Utils.isSimulation()){
            var talonFXSim = m_hoodMotor.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(MotorType.KrakenX60);
        }
    }
    /**
     * Gets turret angle
     * @return radians
     */
    public double getAngle() {
        return m_motorSimModel.getAngle() * k_gearRatio;
    }

    public Command pitchHood(DoubleSupplier distanceToTarget) {
        return Commands.run(() -> {
            m_hoodMotor.setControl(m_request.withPosition(m_hoodLerpTable.get(distanceToTarget.getAsDouble())));
            
        }, this);
    }

    public Command returnHood(){
        return Commands.run(
            () -> m_hoodMotor.setControl(m_request.withPosition(0)), this);
    }

    public Command staticShoot(){
        return Commands.run(
            () -> {
                m_request.Position = ShooterConstants.k_staticShotAngle;
            m_hoodMotor.setControl(m_request);
            }, this);
    }



    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_hoodMotor.getSimState();

   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   m_motorSimModel.setInputVoltage(m_hoodMotor.getMotorVoltage().getValueAsDouble());
   m_motorSimModel.update(0.020);

   talonFXSim.setRawRotorPosition(Units.rotationsToRadians(m_motorSimModel.getAngle()) * k_gearRatio);
   talonFXSim.setRotorVelocity(Units.radiansToRotations(m_motorSimModel.getVelocity()) * k_gearRatio);

   SmartDashboard.putNumber("Hood motor position", Units.radiansToDegrees(m_motorSimModel.getAngle()));

    }
}
