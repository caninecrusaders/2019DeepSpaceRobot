package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class cmdElevatorXbox extends Command {
    public int i = 0;

    public cmdElevatorXbox() {
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.elevator.elevatorXbox();
        // if (Robot.wrist.rangeFinder.getAverageVoltage() < captureDistance) {
        // Robot.oi.xboxDriver.setRumble(RumbleType.kLeftRumble, 1);
        // } else {
        // Robot.oi.xboxDriver.setRumble(RumbleType.kLeftRumble, 0);
        // }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
