package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.driveSystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;

/**
 *
 */
public class cmdStopOnLine extends Command {
    public boolean isStopped = false;
    double speed;
    Counter opticalSensorFront;
    Counter opticalSensorMiddle;
    int step;

    public cmdStopOnLine(double s) {
        speed = s;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveSystem);
        // opticalSensorFront = new Counter(RobotMap.opticalFront);
        // opticalSensorMiddle = new Counter(RobotMap.opticalMiddle);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        opticalSensorFront.reset();
        opticalSensorMiddle.reset();
        isStopped = false;
        step = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (step == 0) { // drive forwards & stop once line is found

            if (opticalSensorFront.get() > 0) {
                System.out.println(opticalSensorFront.get());
                step++;
                opticalSensorFront.reset();
            } else {
                Robot.driveSystem.arcadeDriveXbox();

            }

        } else if (step == 1) { // going backwards until middle sensor detects line
            System.out.println(opticalSensorMiddle.get());
            if (opticalSensorMiddle.get() == 2) {
                isStopped = true;
                step++;
                opticalSensorMiddle.reset();

            } else {
                Robot.driveSystem.drive(-0.1);

            }
        } else if (step == 2) { // turning 90 degrees depending on which bumber is held
            Robot.driveSystem.stop();
            System.out.println("step Two");
            opticalSensorMiddle.reset();
            opticalSensorFront.reset();
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

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
