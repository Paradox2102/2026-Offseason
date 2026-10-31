package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.system.Timer;

import frc.robot.Constants.StallConstants;

public class ParadoxTalon extends TalonFX {
    private static final List<ParadoxTalon> s_all = new ArrayList<>();

    private final BaseStatusSignal m_statorCurrent = getStatorCurrent();
    private final BaseStatusSignal m_velocity = getVelocity();
    private final BaseStatusSignal m_position = getPosition();
    private final Optional<Debouncer> m_stallDebounce;
    private final Timer m_cooldown = new Timer();
    private final NeutralOut m_neutral = new NeutralOut();
    private ControlRequest m_request;
    private boolean m_manualShutoff = false;
    private boolean m_stallShutoff = false;

    private Runnable m_callback;


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
        m_stallDebounce = Optional.of(new Debouncer(stallShutDownTime, DebounceType.RISING));
        setControl(request);

        // Alert on the dashboard if this motor drops off CAN or reboots
        HealthMonitor.register(this);
        s_all.add(this);
    }

    /**
     * Constructs a TalonFX with a few extra utility things. This constructor doesn't run the anti-stall
     * @param deviceId CANID of the motor
     * @param canbus SystemCore Canbus the motor is on
     * @param request The request used to control the motor when motor.applyRequest() is called
     */
    public ParadoxTalon(int deviceId, CANBus canbus, ControlRequest request) {
        super(deviceId, canbus);

        m_request = request;
        m_stallDebounce = Optional.empty();
        setControl(request);

        // Alert on the dashboard if this motor drops off CAN or reboots
        HealthMonitor.register(this);
        s_all.add(this);
    }

    public void setShutoffCallback(Runnable callback) {
        m_callback = callback;
    }

    public static void updateAll() {
        for (ParadoxTalon talon : s_all) {
            talon.updateStall();
        }
    }

    private void updateStall() {
        if (m_stallDebounce.isPresent()){
            BaseStatusSignal.refreshAll(m_statorCurrent, m_velocity, m_position);
        
            if (m_stallShutoff) {
                if (m_cooldown.hasElapsed(StallConstants.k_cooldownSeconds)) {
                    clearStall();
                }
                return;
            }

            boolean stalling = Math.abs(m_statorCurrent.getValueAsDouble()) > StallConstants.k_stallCurrentAmps
                && Math.abs(m_velocity.getValueAsDouble()) < StallConstants.k_stallVelocityRps
                && !isHoldingSetpoint();
            if (m_stallDebounce.get().calculate(stalling)) {
                m_stallShutoff = true;
                m_cooldown.restart();
                if (m_callback != null) {
                    m_callback.run();
                } else {
                    setControl(m_neutral); // actually stop the motor
                }
            }
        }
        
    }

    private boolean isHoldingSetpoint() {
        return m_request instanceof PositionVoltage position
            && Math.abs(m_position.getValueAsDouble() - position.Position) < StallConstants.k_atSetpointToleranceRotations;
    }

    public void setShutoff(boolean shutoff) {
        m_manualShutoff = shutoff;
        applyRequest();
    }

    public boolean getShutoff() {
        return m_manualShutoff || m_stallShutoff;
    }

    public boolean isStallShutoff() {
        return m_stallShutoff;
    }

    public void clearStall() {
        m_stallDebounce.ifPresent(debounce -> debounce.calculate(false)); // reset, so it needs a fresh stall to trip again
        if (m_stallShutoff) {
            m_stallShutoff = false;
            applyRequest();
        }
    }

    public void setParadoxControl(ControlRequest request) {
        m_request = request; // remember it even while shut off, so the motor resumes it after.
        applyRequest();
    }

    /** Send the last request, or neutral while shut off. */
    private void applyRequest() {
        setControl(getShutoff() ? m_neutral : m_request);
    }
}
