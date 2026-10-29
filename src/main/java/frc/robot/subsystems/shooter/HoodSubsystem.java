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
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class HoodSubsystem extends SubsystemBase{
    private final CANBus k_CANBus = CANIDConstants.k_canBus3;

    private final PositionVoltage m_positionRequest = new PositionVoltage(0).withEnableFOC(true);
    private final StaticBrake m_brakeRequest = new StaticBrake();

    private final ParadoxTalon m_hoodMotor = new ParadoxTalon(CANIDConstants.hood, k_CANBus, m_brakeRequest, 0.1);

    public final InterpolatingDoubleTreeMap m_hoodLerpTable = new InterpolatingDoubleTreeMap();

    private final double k_gearRatio = 1;
    private final SingleJointedArmSim m_motorSimModel = new SingleJointedArmSim(DCMotor.getKrakenX44(1),
    k_gearRatio, ShooterConstants.k_hoodMOI, ShooterConstants.k_hoodArmLength, Math.toRadians(0), 8.310546875, true, 0.1);
    private final TunableDouble m_shotAngle = Tunables.addDouble("Hood Angle", 0);

    private DoubleSupplier m_hoodTargetDistance;

    public boolean isOnTarget() {
        // System.out.println("My target: " + String.valueOf(m_positionRequest.Position));
        // System.out.println("My Position: " + String.valueOf(m_hoodMotor.getPosition().getValueAsDouble()));
        return Math.abs(m_hoodMotor.getPosition().getValueAsDouble() - m_positionRequest.Position) < 0.02;
    }

    public HoodSubsystem(DoubleSupplier targetDistance) {
        
        for (double[] shotAngle : ShooterConstants.k_scoreHoodAngle){
            m_hoodLerpTable.put(shotAngle[0], shotAngle[1]);
        }
        m_hoodMotor.getConfigurator().apply(ShooterConstants.k_hoodConfig);
        // m_hoodMotor.setPosition(0);
        if (Utils.isSimulation()){
            var talonFXSim = m_hoodMotor.getSimState();
            talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
            talonFXSim.setMotorType(MotorType.KrakenX60);
        }
        m_hoodTargetDistance = targetDistance;
    }
    /**
     * Gets turret angle
     * @return radians
     */
    public double getAngle() {
        return m_motorSimModel.getAngle() * k_gearRatio;
    }

    public Command pitchHood() {
        return Commands.run(() -> {
            m_hoodMotor.setParadoxControl(m_positionRequest.withPosition(m_hoodLerpTable.get(m_hoodTargetDistance.getAsDouble())));
            
        }, this);
    }

    public Command returnHood(){
        return Commands.runEnd(
            () -> {
                m_hoodMotor.setParadoxControl(m_positionRequest.withPosition(0));
            }, () -> {
                m_hoodMotor.setParadoxControl(m_brakeRequest);
            }, this);
    }

    public Command shotTuning(){
        return Commands.runEnd(
            () -> {
                m_hoodMotor.setParadoxControl(m_positionRequest.withPosition(m_shotAngle.get()));
            }, () -> {
                m_hoodMotor.setParadoxControl(m_brakeRequest);
            }, this);
    }

    public Command staticPitch(){
        return Commands.runEnd(
            () -> {
                m_hoodMotor.setParadoxControl(m_positionRequest.withPosition(ShooterConstants.k_staticShotAngle));
            }, () -> {
                m_hoodMotor.setParadoxControl(m_brakeRequest);
            }, this);
    }

    public Command runPower(){
        return Commands.run(() -> {
            m_hoodMotor.setThrottle(0.5);
        }, this);
    }

    @Override
    public void periodic() {
        Telemetry.log("Hood Output", m_hoodMotor.getClosedLoopOutput().getValueAsDouble());
        Telemetry.log("Hood Position", m_hoodMotor.getPosition().getValueAsDouble());
        Telemetry.log("Hood Target", m_shotAngle);
        Telemetry.log("Hood Target Angle", m_hoodLerpTable.get(m_hoodTargetDistance.getAsDouble()));
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
