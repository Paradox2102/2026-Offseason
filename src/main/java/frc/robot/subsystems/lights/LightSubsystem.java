// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import java.util.Optional;

import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;

public class LightSubsystem extends SubsystemBase {
  /** Creates a new LightSubsystem. */
  private Timer m_timer = new Timer();

  private final double k_shift1Time = 10.0;
  private final double k_shift2Time = 35.0;
  private final double k_shift3Time = 60.0;
  private final double k_shift4Time = 85.0;
  private final double k_endShiftTime = 110.0;
  private final double k_matchEndTime = 140.0;

  private boolean m_autoWinOverride = false;
  private boolean m_overrideWon = false;

  private int m_shift = 0;
  private Optional<String> gameData;



  private final class ShiftCommand extends Command {

    private final int m_commandShift;
    
    public ShiftCommand(int shift) {
      m_commandShift = shift;
    }
    
    @Override
    public void initialize() {
      System.out.println("initialize");
      if (m_commandShift == 0){
        m_timer.reset();
      }
      m_timer.start();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_shift ++;
    }
    public void putShiftTime(double shift) {
      Telemetry.log("Time Until Shift", round(shift - m_timer.get()));
    }
    public double round(double number) {
      double biigerNumber = Math.floor(number * 10);
      return (biigerNumber) / 10;
    }
    @Override
    public boolean isFinished() {
      switch(m_commandShift) {
        case 0:
          putShiftTime(k_shift1Time);
          return m_timer.get() > k_shift1Time;
          
        case 1:
          putShiftTime(k_shift2Time);
          return m_timer.get() > k_shift2Time;

        case 2:
          putShiftTime(k_shift3Time);
          return m_timer.get() > k_shift3Time;

        case 3:
          putShiftTime(k_shift4Time);
          return m_timer.get() > k_shift4Time;

        case 4:
          putShiftTime(k_endShiftTime);
          return m_timer.get() > k_endShiftTime;

        case 5:
          putShiftTime(k_matchEndTime);
          return m_timer.get() > k_matchEndTime;

        default:
          return true;

      }
    }
  }

  public LightSubsystem() {
   
    new Trigger(() -> RobotState.isTeleop() && !RobotState.isDisabled()).onTrue(new ShiftCommand( 0));
    new Trigger(() -> m_timer.get() > k_shift1Time).onTrue(new ShiftCommand(1));
    new Trigger(() -> m_timer.get() > k_shift2Time).onTrue(new ShiftCommand(2));
    new Trigger(() -> m_timer.get() > k_shift3Time).onTrue(new ShiftCommand(3));
    new Trigger(() -> m_timer.get() > k_shift4Time).onTrue(new ShiftCommand(4));
    new Trigger(() -> m_timer.get() > k_endShiftTime).onTrue(new ShiftCommand(5));
  }

  @Override
  public void periodic() {
  gameData = MatchState.getGameData();
    Telemetry.log("Won Auto?", getWonAuto());
  }
  /**
   * Using the current shift, estimated through a timer, It will return if the alliance hub is currently active.
   * @return Boolean returning the current active hub
   */
  public boolean hubIsActive(){
    switch (m_shift) {
      case 0:
        return true;
      case 1:
        return !getWonAuto();
      case 2:
        return getWonAuto();
      case 3:
        return !getWonAuto();
      case 4:
        return getWonAuto();
      case 5:
        return true;
      default:
        return false;
    }
  }
  /**
   * It gets the game specific message and using that determines what side won auto.
   * Note: This is untested since we don't have a driver station for testing.
   * @return Boolean returning whether your current alliance won Auto
   */
  public boolean getWonAuto() {
    if (m_autoWinOverride){
      return m_overrideWon;
    }
    if (gameData.isPresent()) {
      switch (gameData.get().charAt(0)) {
        case 'B':
          if (MatchState.getAlliance().get() == Alliance.BLUE) {
            return true;
          }
          else {
            return false;
          }
        case 'R':
          if (MatchState.getAlliance().get() == Alliance.RED) {
            return true;
          }
          else {
            return false;
          }
        default:
          return true;
      }
    }
    else {
      return false;
    }
  }

  public Command overrideWonAuto(boolean wonAuto){
    return Commands.runOnce(() -> {
      m_autoWinOverride = true;
      m_overrideWon = wonAuto;
    });
  }
}
