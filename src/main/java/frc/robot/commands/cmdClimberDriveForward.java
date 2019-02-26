/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class cmdClimberDriveForward extends Command {
  public cmdClimberDriveForward() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSystem);
    setTimeout(12.0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSystem.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.climber.rangeToFloor.getRangeInches() > 4) {
      Robot.driveSystem.drive(0.3);
    } else {
      Robot.driveSystem.stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (isTimedOut()/* || Robot.climber.rangeToFloor.getRangeInches() < 4.0 */) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveSystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
