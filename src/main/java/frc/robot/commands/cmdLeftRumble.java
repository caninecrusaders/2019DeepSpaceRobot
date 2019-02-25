/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class cmdLeftRumble extends Command {
  boolean isDriver1;
  double rumble;
  double timeout;

  public cmdLeftRumble(boolean _isDriver1, double _rumble, double _timeout) {
    // Use requires() here to declare subsystem dependencies
    this.isDriver1 = _isDriver1;
    this.rumble = _rumble;
    this.timeout = _timeout;
    requires(Robot.rumble);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(timeout);
    Robot.rumble.leftRumble(this.isDriver1, this.rumble);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Robot.rumble.leftRumble(this.isDriver1, this.rumble);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.rumble.stopRumble(this.isDriver1);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
