package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class cmdWristXbox extends Command {
    public double captureDistance = 0.4;
    public double releaseDistance = 3.0;

    public cmdWristXbox() {
        requires(Robot.wrist);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double intakeIn = -Robot.oi.xboxDriver2.getRawAxis(1);
        double intakeOut = -intakeIn;
        if (intakeIn > 0/* && Robot.wrist.rangeFinder.getAverageVoltage() > captureDistance */) {
            Robot.wrist.intakeIn(intakeIn);
        } else if (intakeOut > 0 /* && Robot.wrist.rangeFinder.getAverageVoltage() < releaseDistance */) {
            Robot.wrist.intakeOut(intakeOut);
        } else {
            Robot.wrist.intakeStop();
        }

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
        Robot.wrist.intakeStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
