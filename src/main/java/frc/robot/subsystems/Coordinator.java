// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.Shoot;
import frc.robot.subsystems.DriveStateMachine.DriveState;
import frc.utils.FlytDashboard;


/** Bridges the drive and manipulator state machines while updating LEDs for feedback. */
public class Coordinator extends SubsystemBase {

  // attributes
  /** Mirrors the robot enable state so LEDs can be updated appropriately. */
  private boolean isDisabled = true;

  /** Indicates whether autonomous mode is active. */
  private boolean isAuto = false;


  /** Active controller profile that dictates button bindings. */
  private ControllerProfile currentProfile = ControllerProfile.ACTIVE;

  /** Latest global goalState requested by the driver or auto routine. */
  private RobotState goalState = RobotState.START_POSITION;

  private DriveStateMachine driveSM;
  private FlytDashboard dashboard = new FlytDashboard("Coordinator");

  // /** Helper for setting global LED patterns based on robot state. */
  // private LEDUtility ledUtility;

  public enum RobotState {
    START_POSITION,
    INTAKE,
    SHOOT,
    CLIMB,
    MANUAL,
    INTAKE_SHOOT,
    RESET;
  }

  public enum ControllerProfile {
    MANUAL, 
    CLIMG, 
    INTACTIVE,
    ACTIVE;
  }

  public Coordinator(DriveStateMachine m_driveStateMachine) {
    driveSM = m_driveStateMachine;

  }

  public void robotDisabled(boolean disabled) {
    isDisabled = disabled;
  }

  public void robotAuto(boolean auto) {
    isAuto = auto;
  }


  /**
   * Returns the current controller profile.
   *
   * @return
   */
  public ControllerProfile getCurrentControllerProfile() {
    return currentProfile;
  }

  public void setControllerProfile(ControllerProfile profile) {
    // Drivers can swap profiles to expose different button mappings and LED themes.
    currentProfile = profile;
    // Prevent stale transitions from leaving the robot in a weird previous state if it gets stuck.
  }

  // Set drive goal.
  private void setDriveGoal(DriveState m_drivestate) {
    driveSM.setDriveCommand(m_drivestate);
  }

  /**
   * Sets the global state goal.
   *
   * @param state
   */
  public void setRobotGoal(RobotState state) {
    // safety checks before requesting state changes
    goalState = state;
    // state change
    switch (state) {
      case START_POSITION:
        setDriveGoal(DriveState.CANCELLED);
        break;
      case MANUAL:
        setDriveGoal(DriveState.MANUAL);
        break;
      case SHOOT:
        // setDriveGoal(DriveState.HUB_ORIENTED);
        setDriveGoal(DriveState.POINT_AT_FUEL);
        break;
      case RESET:
        setDriveGoal(DriveState.CANCELLED);
        break;
    }
  }

  public void requestToScore(boolean score) {
    //manipulatorSM.score(score);
  }

  /** Handles automatic state transitions based on current subsystem states. */
  private void handleAutomaticTransitions() {


    
    //leds stuff
    if (false) {
    //   if (isAuto) {
    //     ledUtility.setAll(LEDEffect.PULSE, LEDEffects.flytBlue);
    //   } else if (isDisabled) {
    //     ledUtility.getStrip("TopLeft").setEffect(LEDEffect.NAVLIGHTS, Color.kRed);
    //     ledUtility.getStrip("TopRight").setEffect(LEDEffect.NAVLIGHTS, Color.kGreen);
    //     ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
    //     ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
    //   } else if (manipulatorSM.getCurrentState() == ElevatorManipulatorState.INTAKE_CORAL
    //       && !gethasCoral()
    //       && driveSM.getCurrentState() == DriveState.CORAL_STATION) {
    //     ledUtility.getStrip("Left").setEffect(LEDEffect.FLASH, Color.kGreen);
    //     ledUtility.getStrip("Right").setEffect(LEDEffect.FLASH, Color.kGreen);
    //   }
    // } else if (manipulatorSM.getCurrentState() == ElevatorManipulatorState.INTAKE_CORAL
    //     && manipulatorSM.getHasCoral()
    //     && driveSM.getCurrentState() == DriveState.CORAL_STATION) {
    //   ledUtility.getStrip("Left").setEffect(LEDEffect.SOLID, Color.kGreen);
    //   ledUtility.getStrip("Right").setEffect(LEDEffect.SOLID, Color.kGreen);
    // } else if (getCurrentControllerProfile() == ControllerProfile.MANUAL) {
    //   ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kRed);
    //   ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kRed);
    //   ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
    //   ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
    // } else if (driveSM.getCurrentState() == DriveState.REEF_RELATIVE) {
    //   if (getRightScore()) {
    //     ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kPurple);
    //     ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kPurple);
    //   } else {
    //     ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kYellow);
    //     ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kYellow);
    //   }
    //   ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
    //   ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
    // } else if (driveSM.getCurrentState() == DriveState.REEF_ALIGN) {
    //   ledUtility.getStrip("Left").setEffect(LEDEffect.FLASH, LEDEffects.flytBlue);
    //   ledUtility.getStrip("Right").setEffect(LEDEffect.FLASH, LEDEffects.flytBlue);
    // } else if (driveSM.atPosition() && manipulatorSM.atGoalState()) {
    //   ledUtility.getStrip("Left").setEffect(LEDEffect.SOLID, Color.kGreen);
    //   ledUtility.getStrip("Right").setEffect(LEDEffect.SOLID, Color.kGreen);
    }
  }

  @Override
  public void periodic() {

    dashboard.putString("Current State", goalState.toString());
    dashboard.putBoolean("RobotDisabled", isDisabled);
    dashboard.putBoolean("Robot is in auto", isAuto);
    dashboard.putString("Controller Profile", getCurrentControllerProfile().toString());
    // This method will be called once per scheduler run
    handleAutomaticTransitions();
  }
}
