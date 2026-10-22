package frc.robot.constants;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;

/**
 * Every CAN ID and bus on the robot, in one place.
 *
 * <p>The SystemCore has five classic CAN buses at 1 Mbit/s, CAN_S0 to CAN_S4 (no CAN FD, no
 * CANivore). A device ID only has to be unique on its own bus, but we keep them unique on the whole
 * robot so a name in a log is never ambiguous.
 */
public final class CanIds {
    private CanIds() {}

    public static final CANBus k_canBus0 = new CANBus(CANPort.CAN_S0); // PDH only
    public static final CANBus k_canBus1 = new CANBus(CANPort.CAN_S1); // indexer
    public static final CANBus k_canBus2 = new CANBus(CANPort.CAN_S2); // swerve: 8 Talons, 4 CANcoders, Pigeon
    public static final CANBus k_canBus3 = new CANBus(CANPort.CAN_S3); // shooter: kickers, hood, flywheel, turret
    public static final CANBus k_canBus4 = new CANBus(CANPort.CAN_S4); // intake: roller and rack

    public static final int gyro = 0;

    // The PDH is alone on CAN_S0 with CAN ID 21 (REV default is 1)
    public static final CANPort k_pdhPort = CANPort.CAN_S0;
    public static final int pdh = 21;

    public static final int fl_drive = 1;
    public static final int fl_steer = 2;
    public static final int fr_drive = 3;
    public static final int fr_steer = 4;
    public static final int bl_drive = 5;
    public static final int bl_steer = 6;
    public static final int br_drive = 7;
    public static final int br_steer = 8;
    public static final int fl_encoder = 9;
    public static final int fr_encoder = 10;
    public static final int bl_encoder = 11;
    public static final int br_encoder = 12;

    public static final int intakeRoller = 20;
    public static final int intakeExtensionLeader = 21;
    public static final int intakeExtensionFollower = 22;

    public static final int indexerRoller = 30;
    public static final int indexerConveyorLeader = 31;
    public static final int indexerConveyorFollower = 32;

    public static final int kicker = 40;
    public static final int hood = 41;
    public static final int shooterLeader = 42;
    public static final int shooterFollower = 43;
    public static final int shooterAzimuth = 44;
    public static final int goUp = 45;
}
