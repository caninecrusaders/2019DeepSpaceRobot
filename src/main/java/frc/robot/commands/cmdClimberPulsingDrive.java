/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class cmdClimberPulsingDrive extends Command {
  double timeStart;
  boolean isMotorOn = false;
  int counter;

  public cmdClimberPulsingDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSystem);
    setTimeout(2.5);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timeStart = Timer.getFPGATimestamp();
    isMotorOn = false;
    counter = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double deltaTime = Timer.getFPGATimestamp() - timeStart;
    if (deltaTime >= 0.1) {
      counter++;
      timeStart = Timer.getFPGATimestamp();
      if (isMotorOn) {
        isMotorOn = false;
      } else {
        isMotorOn = true;
      }
    }

    // if (Robot.driveSystem.rangeInFront.getRangeInches() < 34.0) {
    if (isMotorOn) {
      Robot.driveSystem.drive(0.2);
    } else {
      Robot.driveSystem.stop();
      ;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
    // if (Robot.driveSystem.rangeInFront.getRangeInches() < 32.0
    // && (Robot.ahrs.getPitch() < 0.02 && Robot.ahrs.getPitch() > -0.02)) {
    // return true;
    // } else {
    // return false;
    // }
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
