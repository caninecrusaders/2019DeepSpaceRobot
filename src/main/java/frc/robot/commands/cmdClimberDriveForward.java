/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class cmdClimberDriveForward extends Command {
  boolean isClimbing = false;
  double time;
  double speed;

  public cmdClimberDriveForward(double timeOut, double speedIn) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSystem);
    speed = speedIn;
    setTimeout(timeOut);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Robot.driveSystem.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if (Robot.climber.rangeToFloor.getRangeInches() > 18 && !isClimbing) {
    // isClimbing = true;
    // }
    // if (isClimbing) {
    // Robot.driveSystem.drive(0.6);
    // } else {
    // Robot.driveSystem.stop();
    // }
    Robot.driveSystem.drive(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (isTimedOut()/* || (Robot.climber.rangeToFloor.getRangeInches() < 4.0 && isClimbing) */) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveSystem.stop();
    // SmartDashboard.putNumber("RangeToFloorOnPlatform",
    // Robot.climber.rangeToFloor.getRangeInches());

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
