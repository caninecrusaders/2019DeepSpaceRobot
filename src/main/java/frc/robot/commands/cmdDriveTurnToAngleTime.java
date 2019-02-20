package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class cmdDriveTurnToAngleTime extends Command {
    double speed;

    public cmdDriveTurnToAngleTime(double timeOutS, double speedIn) {
        speed = speedIn;
        requires(Robot.driveSystem);
        setTimeout(timeOutS);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.driveSystem.rotate(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (isTimedOut()) {
            return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.driveSystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
