/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class cmdIntakeDown extends Command {
  public cmdIntakeDown() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.wrist);
    requires(Robot.elbow);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (Robot.elevator.isBallMode()) {
      Robot.wrist.wristUp();
      Robot.elbow.ElbowDown();
    } else {
      Robot.wrist.wristDown();
      Robot.elbow.ElbowDown();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!Robot.elevator.isBallMode()) {
      Robot.wrist.wristUp();
      Robot.elbow.ElbowDown();
    } else {
      Robot.wrist.wristDown();
      Robot.elbow.ElbowDown();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
