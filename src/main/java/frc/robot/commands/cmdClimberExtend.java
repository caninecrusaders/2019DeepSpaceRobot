/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class cmdClimberExtend extends Command {
  double startTime;

  public cmdClimberExtend() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
    Robot.climber.startingPitch = Robot.ahrs.getPitch();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.climber.down();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double currentPitch = Robot.ahrs.getPitch();
    if (Robot.climber.startingPitch - currentPitch > 0.5) {
      Robot.climber.timeExtend = Timer.getFPGATimestamp() - startTime;
      // SmartDashboard.putNumber("lastPitch", currentPitch);
      // SmartDashboard.putNumber("startingPitch", Robot.climber.startingPitch);
      // SmartDashboard.putNumber("heightAtTrigger",
      // Robot.climber.rangeToFloor.getRangeInches());

      return true;
    }
    return false;
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
