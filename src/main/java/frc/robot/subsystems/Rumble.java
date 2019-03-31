/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Rumble extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new cgBallRumble());
  }

  public void leftRumble(boolean isDriver1, double rumble) {
    if (isDriver1) {
      Robot.oi.xboxDriver2.setRumble(RumbleType.kLeftRumble, rumble);
    } else {
      Robot.oi.xboxDriver2.setRumble(RumbleType.kLeftRumble, rumble);
    }

  }

  public void rightRumble(boolean isDriver1, double rumble) {
    if (isDriver1) {
      Robot.oi.xboxDriver2.setRumble(RumbleType.kRightRumble, rumble);
    } else {
      Robot.oi.xboxDriver2.setRumble(RumbleType.kRightRumble, rumble);
    }

  }

  public void stopRumble(boolean isDriver1) {
    leftRumble(isDriver1, 0);
    rightRumble(isDriver1, 0);
  }
}
