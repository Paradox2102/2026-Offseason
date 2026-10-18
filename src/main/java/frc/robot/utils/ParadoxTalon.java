package frc.robot.utils;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

public class ParadoxTalon extends TalonFX {
    private ControlRequest m_request;
    private final double k_stallShutDownTime;

    /**
     * Constructs a TalonFX with a few extra utility things
     * @param deviceId CANID of the motor
     * @param canbus SystemCore Canbus the motor is on
     * @param request The request used to control the motor when motor.applyRequest() is called
     * @param stallShutDownTime How long the motor should stall for before it shuts down for a few seconds.
     */
    public ParadoxTalon(int deviceId, CANBus canbus, ControlRequest request, double stallShutDownTime) {
        super(deviceId, canbus);

        m_request = request;
        k_stallShutDownTime = stallShutDownTime;
        setControl(request);

        // Alert on the dashboard if this motor drops off CAN or reboots
        HealthMonitor.register(this);
    }
 
    
}
