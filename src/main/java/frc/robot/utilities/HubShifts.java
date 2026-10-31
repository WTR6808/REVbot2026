package frc.robot.utilities;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// Copyright (c) 2025-2026 Littleton Robotics
// https://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Modified by William Tennent Robotics, team 6808

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubShifts extends SubsystemBase{
    private static HubShifts instance = null;

    public static HubShifts getInstance(){
        if(instance == null){
            instance = new HubShifts();
        }
        return instance;
    }

  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED;
  }

  public record ShiftInfo(
      ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

  private static Timer shiftTimer = new Timer();
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  private static final double minFuelCountDelay = 1.0;
  private static final double maxFuelCountDelay = 2.0;
  private static final double shiftEndFuelCountExtension = 3.0;
  private static final double minTimeOfFlight = 1.5; //TODO calculate
  private static final double maxTimeOfFlight = 5.0; //TODO calculate
  private static final double approachingActiveFudge = -1 * (minTimeOfFlight + minFuelCountDelay);
  private static final double endingActiveFudge =
      shiftEndFuelCountExtension + -1 * (maxTimeOfFlight + maxFuelCountDelay);

  public static final double autoEndTime = 20.0;
  public static final double teleopDuration = 140.0;
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};


  public static Alliance getFirstActiveAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Return FMS value
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    // Return default value
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  /** Starts the timer at the begining of teleop. */
  public static void initialize() {
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    boolean[] currentSchedule;
    Alliance startAlliance = getFirstActiveAlliance();
    currentSchedule =
        startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
            ? activeSchedule
            : inactiveSchedule;
    return currentSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
    double currentTime = shiftTimer.get();
    double stateTimeElapsed = shiftTimer.get();
    double stateTimeRemaining = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isEnabled()) {
      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        // After last shift, so assume endgame
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      // If the state is the same as the last shift, combine the elapsed time
      if (currentShiftIndex > 0) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
          stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
        }
      }

      // If the state is the same as the next shift, combine the remaining time
      if (currentShiftIndex < shiftEndTimes.length - 1) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
          stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
        }
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = shiftsEnums[currentShiftIndex];
    }
    ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
    return shiftInfo;
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] shiftSchedule = getSchedule();
    // Starting active
    if (shiftSchedule[1] == true) {
      double[] shiftedShiftStartTimes = {
        0.0,
        10.0,
        35.0 + endingActiveFudge,
        60.0 + approachingActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + approachingActiveFudge
      };
      double[] shiftedShiftEndTimes = {
        10.0,
        35.0 + endingActiveFudge,
        60.0 + approachingActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + approachingActiveFudge,
        140.0
      };
      return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    }
    double[] shiftedShiftStartTimes = {
      0.0,
      10.0 + endingActiveFudge,
      35.0 + approachingActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + approachingActiveFudge,
      110.0
    };
    double[] shiftedShiftEndTimes = {
      10.0 + endingActiveFudge,
      35.0 + approachingActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + approachingActiveFudge,
      110.0,
      140.0
    };
    return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    // }
  }

  @Override
    public void periodic() {
      ShiftInfo info = getOfficialShiftInfo();
      SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
      SmartDashboard.putNumber("Remaining Shift Time", Math.round(info.remainingTime() * 10.0)/10.0);
      SmartDashboard.putBoolean("Shift Active", info.active());
      SmartDashboard.putString("Game State", info.currentShift().toString());
      SmartDashboard.putBoolean("Active First?",
      getFirstActiveAlliance() == DriverStation.getAlliance().orElse(Alliance.Blue));
    }
}
