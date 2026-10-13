package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.ParadoxTalon;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class HoodSubsystem extends SubsystemBase{
    private final CANBus k_CANBus = CANIDConstants.k_canBus3;

    private final PositionVoltage m_positionRequest = new PositionVoltage(0);
    private final StaticBrake m_brakeRequest = new StaticBrake();

    private final ParadoxTalon m_hoodMotor = new ParadoxTalon(CANIDConstants.hood, k_CANBus, m_brakeRequest, 0.1);

    private final InterpolatingDoubleTreeMap m_hoodLerpTable = new InterpolatingDoubleTreeMap();

    private final double k_gearRatio = 1;

    private final SingleJointedArmSim m_motorSimModel = new SingleJointedArmSim(DCMotor.getKrakenX44(1),
    k_gearRatio, ShooterConstants.k_hoodMOI, ShooterConstants.k_hoodArmLength, Math.toRadians(0), Math.toRadians(42), true, 0.1);
    
    public boolean m_isOnTarget() {
        return Math.abs(m_hoodMotor.getPosition().getValueAsDouble() - m_positionRequest.Position) < 0.1;
    }

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
            m_hoodMotor.setRequest(m_positionRequest.withPosition(m_hoodLerpTable.get(distanceToTarget.getAsDouble())));
            m_hoodMotor.applyRequest(); 
        }, this);
    }

    public Command returnHood(){
        return Commands.runEnd(
            () -> {
                m_hoodMotor.setRequest(m_positionRequest.withPosition(0));
                m_hoodMotor.applyRequest();
            }, () -> {
                m_hoodMotor.setRequest(m_brakeRequest);
                m_hoodMotor.applyRequest();
            }, this);
    }

    public Command staticPitch(){
        return Commands.runEnd(
            () -> {
                m_hoodMotor.setRequest(m_positionRequest.withPosition(ShooterConstants.k_staticShotAngle));
                m_hoodMotor.applyRequest();
            }, () -> {
                m_hoodMotor.setRequest(m_brakeRequest);
                m_hoodMotor.applyRequest();
            }, this);
    }



    @Override
    public void simulationPeriodic() {
    var talonFXSim = m_hoodMotor.getSimState();

   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   m_motorSimModel.setInputVoltage(talonFXSim.getMotorVoltage());
   m_motorSimModel.update(0.020);

   talonFXSim.setRawRotorPosition(Units.radiansToRotations(m_motorSimModel.getAngle()) * k_gearRatio);
   talonFXSim.setRotorVelocity(Units.radiansToRotations(m_motorSimModel.getVelocity()) * k_gearRatio);

   Telemetry.log("Hood motor position", Units.radiansToDegrees(m_motorSimModel.getAngle()));

    }
}
