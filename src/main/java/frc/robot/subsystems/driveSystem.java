// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

//import java.io.Console;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;

//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Joystick;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class driveSystem extends Subsystem implements PIDOutput {
	public boolean invert = false;
	public boolean inVisionMode = false;
	public double visionAngle;
	public boolean isVisionAngleValid = false;
	PIDController controller;
	double rotateToAngleRate;
	static final double kP = 0.02; // 0.015
	static final double kI = 0.001;
	static final double kD = 0.045; // 0.03
	static final double kF = 0.1;
	static final double kToleranceDegrees = 2.0f;
	double last_world_linear_accel_x;
	double last_world_linear_accel_y;
	final static double kCollisionThreshold_DeltaG = 0.6f;
	public Ultrasonic rangeInFront = new Ultrasonic(RobotMap.frontTriggerID, RobotMap.frontEchoID);
	public Ultrasonic rangeInBack = new Ultrasonic(RobotMap.backTriggerID, RobotMap.backEchoID);
	static double lastThrottleTime;

	public WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(RobotMap.driveFrontLeftMotorID);
	public WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(RobotMap.driveFrontRightMotorID);
	public WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(RobotMap.driveBackLeftMotorID);
	public WPI_TalonSRX backRightMotor = new WPI_TalonSRX(RobotMap.driveBackRightMotorID);
	SpeedControllerGroup leftMotors;
	SpeedControllerGroup rightMotors;
	DifferentialDrive driveControl;
	double deadZone = 0.15;

	boolean stop = false;
	boolean isRotating = false;

	public driveSystem() {
		setUpPIDController();
		frontLeftMotor.setInverted(true);
		frontRightMotor.setInverted(true);
		backLeftMotor.setInverted(true);
		backRightMotor.setInverted(true);
		LiveWindow.add(frontLeftMotor);
		LiveWindow.add(frontRightMotor);
		LiveWindow.add(backLeftMotor);
		LiveWindow.add(backRightMotor);
		leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
		rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);
		driveControl = new DifferentialDrive(leftMotors, rightMotors);
		LiveWindow.add(driveControl);
		LiveWindow.add(controller);
	}

	public void setUpPIDController() {
		controller = new PIDController(kP, kI, kD, kF, Robot.ahrs, this);
		controller.setInputRange(-180.0f, 180.0f);
		controller.setOutputRange(-0.4, 0.4);
		controller.setAbsoluteTolerance(kToleranceDegrees);
		controller.setContinuous(true);
	}

	public void enablePIDController(double setpoint) {
		controller.reset();
		controller.setSetpoint(setpoint);
		controller.enable();
	}

	public void disablePIDController() {
		controller.disable();
	}

	@Override
	public void initDefaultCommand() {
		rangeInFront.setAutomaticMode(true);
		// setDefaultCommand(new cmdDriveXbox());
		setDefaultCommand(new cmdCurvatureDriveXbox());

	}

	@Override
	public void periodic() {
		// Put code here to be run every loop

	}

	public void tankDriveXbox() {
		if (turnToAngleOnJoystick()) {
			return;
		}
		double left = Robot.oi.xboxDriver.getRawAxis(1);
		double right = Robot.oi.xboxDriver.getRawAxis(5);
		driveControl.tankDrive(left, right);
	}

	public void arcadeDriveXbox() {
		if (turnToAngleOnJoystick()) {
			return;
		}
		double throttle = Robot.oi.xboxDriver.getRawAxis(1);
		double turn = Robot.oi.xboxDriver.getRawAxis(4);
		driveControl.arcadeDrive(throttle, -turn);
	}

	public void curvatureDriveXbox() {
		if (turnToAngleOnJoystick()) {
			return;
		}
		double throttle = Robot.oi.xboxDriver.getRawAxis(1);
		double turn = Robot.oi.xboxDriver.getRawAxis(4);
		// check to see if were in vision mode
		if (inVisionMode) {
			if (isVisionAngleValid) {
				turn = Math.abs(visionAngle) / 160.0;
				turn = Math.min(turn, 1.0);
				turn = Math.copySign(turn, visionAngle);
			} else {
				turn = 0;
			}
		}
		// driveControl.curvatureDrive(throttle, -turn, false);
		if (throttle > -.1 && throttle < 0.1 && !inVisionMode) {
			if (Timer.getFPGATimestamp() - lastThrottleTime > 0.25) {
				driveControl.curvatureDrive(0, -turn * 0.6, true);
			}
		} else {
			driveControl.curvatureDrive(throttle, -turn, false);
			lastThrottleTime = Timer.getFPGATimestamp();
		}
	}
	// public void tankDriveXbox() {
	// double left = -Robot.oi.xboxDriver.getRawAxis(5);
	// double right = -Robot.oi.xboxDriver.getRawAxis(1);
	// double straightTolerance = 0.5;
	// if (Math.abs(left - right) <= straightTolerance) {
	// left = right = (left + right) / 2.0;
	// }
	// if (invert) {
	// // Invert signal
	// left = -left;
	// right = -right;
	// }
	// if (Math.abs(right) < deadZone) {
	// right = 0.0;
	// }
	// if (Math.abs(left) < deadZone) {
	// left = 0.0;
	// }
	// Robot.driveSystem.frontRightMotor.set(right);
	// frontLeftMotor.set(-left);
	// frontRightMotor.set(-right);
	// backRightMotor.set(-right);
	// backLeftMotor.set(-left);
	// }

	// public void arcadeDriveXbox() {
	// if (turnToAngleOnJoystick()) {
	// return;
	// }
	// double throttle = Robot.oi.xboxDriver.getRawAxis(1);
	// double turn = -Robot.oi.xboxDriver.getRawAxis(4);
	// double leftMotorSpeed = 0;
	// double rightMotorSpeed = 0;
	// double deadZone = 0.15;

	// if (Math.abs(throttle) < deadZone) {
	// throttle = 0.0;
	// }
	// if (Math.abs(turn) < deadZone) {
	// turn = 0.0;
	// }

	// if (throttle > deadZone) {
	// if (turn > deadZone) {
	// leftMotorSpeed = throttle - turn;
	// rightMotorSpeed = Math.max(throttle, -turn);
	// } else {
	// leftMotorSpeed = throttle + turn;
	// rightMotorSpeed = throttle + turn;
	// }
	// } else {
	// if (turn > deadZone) {
	// leftMotorSpeed = -Math.max(-throttle, turn);
	// rightMotorSpeed = throttle + turn;
	// } else {
	// leftMotorSpeed = throttle - turn;
	// rightMotorSpeed = -Math.max(-throttle, -turn);
	// }
	// }
	// // if (stop || RobotMap.optical.get()){
	// // leftMotorSpeed = rightMotorSpeed = 0;
	// // stop = true;
	// // }
	// // SmartDashboard.putNumber("leftSpeed", leftMotorSpeed);
	// // SmartDashboard.putNumber("rightSpeed", rightMotorSpeed);
	// frontLeftMotor.set(ControlMode.PercentOutput, leftMotorSpeed);
	// frontRightMotor.set(ControlMode.PercentOutput, rightMotorSpeed);
	// backLeftMotor.set(ControlMode.PercentOutput, leftMotorSpeed);
	// backRightMotor.set(ControlMode.PercentOutput, rightMotorSpeed);

	// }

	public void rotate(double speed) {
		// driveControl.tankDrive(speed, -speed);
		driveControl.curvatureDrive(0, speed, true);
	}

	public void breaking(Boolean breaksOn) {

		// frontRightMotor.set(ControlMode.PercentOutput, -speed);
		// backRightMotor.set(ControlMode.PercentOutput, -speed);
		/// backLeftMotor.set(ControlMode.PercentOutput, speed);
	}

	public boolean turnToAngleOnJoystick() {
		int dPadValue = Robot.oi.xboxDriver.getPOV();
		if (dPadValue == -1) {
			disablePIDController();
			isRotating = false;
			// Robot.oi.xboxDriver.setRumble(RumbleType.kLeftRumble, 0);
			return false;
		}
		if (dPadValue == 45) {
			dPadValue = 61;
		} else if (dPadValue == 135) {
			dPadValue = 119;
		} else if (dPadValue == 225) {
			dPadValue = -61;
		} else if (dPadValue == 315) {
			dPadValue = -119;
		}
		if (dPadValue > 180) {
			dPadValue = dPadValue - 360;
		}

		if (isRotating) {
			rotateToAngle(dPadValue);
			if (isOnTargetAngle()) {
				Robot.oi.xboxDriver.setRumble(RumbleType.kLeftRumble, 1);
			} else {
				Robot.oi.xboxDriver.setRumble(RumbleType.kLeftRumble, 0);
			}

		} else {
			isRotating = true;
			enablePIDController(dPadValue);
		}
		return true;
	}

	public void rotateToAngle(double angle) {
		rotate(-rotateToAngleRate);
		// controller.setSetpoint(angle);
	}

	public boolean isOnTargetAngle() {
		return controller.onTarget();
	}

	public void curve(double leftSpeed, double rightSpeed, double angle) {

		frontLeftMotor.set(leftSpeed);
		backLeftMotor.set(leftSpeed);
		frontRightMotor.set(rightSpeed);
		backRightMotor.set(rightSpeed);

	}

	// Current Time = t
	// Start Value = b
	// Change in value = c
	// Duration = d
	public double easeIn(double t, double b, double c, double d) {
		t = t / d;
		return c * t * t + b;
	}

	public double easeOut(double t, double b, double c, double d) {
		t = t / d;
		return -c * t * (t - 2.0) + b;
	}

	public double easeInOut(double t, double b, double c, double d) {
		t /= d / 2.0;
		if (t < 1) {
			return c / 2.0 * t * t + b;
		}
		return -c / 2.0 * ((--t) * (t - 2) - 1) + b;
	}

	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}

	public boolean pidOnTarget() {
		return controller.onTarget();
	}

	public void drive(double speed) {
		driveControl.curvatureDrive(-speed, 0, false);

		// frontLeftMotor.set(speed);
		// backLeftMotor.set(speed);
		// frontRightMotor.set(speed);
		// backRightMotor.set(speed);
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public void stop() {
		driveControl.curvatureDrive(0, 0, false);
		// frontLeftMotor.set(0);
		// backLeftMotor.set(0);
		// frontRightMotor.set(0);
		// backRightMotor.set(0);
	}

	public void driveForward(double speed, double angle) {
		if (!isRotating) {
			enablePIDController(angle);
			isRotating = true;
		}
		driveControl.curvatureDrive(-speed, -rotateToAngleRate, true);
	}

}
