/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class cmdClimberRetract extends Command {
  double time;
  double speed;

  // double timeStart;
  public cmdClimberRetract(double Time, double Speed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climber);
    // setTimeout(Timer.getFPGATimestamp() + Robot.climber.timeExtend + 3.0);
    time = Time;
    speed = Speed;
    setTimeout(time);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // timeStart = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if (Robot.ahrs.getPitch() < 0.02 && Robot.ahrs.getPitch() > -0.02) {
    // Robot.climber.stop();
    // } else {
    Robot.climber.up(speed);
    // }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // double deltaTime = time - timeStart;
    // if (deltaTime == time) {
    // return true;
    // } else {
    // return false;
    // }
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climber.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
