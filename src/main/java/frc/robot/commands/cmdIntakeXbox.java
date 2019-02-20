/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class cmdIntakeXbox extends Command {
  public int i = 0;

  public cmdIntakeXbox() {
    requires(Robot.wrist);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // double out = Robot.oi.xboxDriver.getRawAxis(2);
    // double in = Robot.oi.xboxDriver.getRawAxis(3);
    // if (in > 0) {
    // Robot.wrist.intakeIn(in);
    // } else if (out > 0) {
    // Robot.wrist.intakeOut(out);
    // } else {
    // Robot.wrist.intakeStop();
    // }
    double out = Robot.oi.xboxDriver2.getRawAxis(1);
    if (out < 0) {
      Robot.wrist.intakeOut(-out);
    } else if (out > 0) {
      Robot.wrist.intakeIn(out);
    } else {
      Robot.wrist.intakeStop();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
