/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;
import frc.robot.commands.cgBallRumble;
import frc.robot.commands.cmdElevatorXbox;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem implements PIDOutput {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  PIDController controller;
  double elevatorSpeed;
  static final double kP = 0.04;
  static final double kI = 0.003;
  static final double kD = 0.002;
  static final double kF = 0.1;
  static final double kToleranceVolts = 0.1;
  double last_world_linear_accel_x;
  double last_world_linear_accel_y;
  final static double kCollisionThreshold_DeltaG = 0.6f;

  private final WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(RobotMap.elevatorMotorID);
  public AnalogInput elevatorPot = new AnalogInput(RobotMap.elevatorPotID);
  private final double[] potHatch = new double[] { 0.0, 1.0, 2.0, 3.0 };
  private final double[] potBall = new double[] { 0.0, 1.0, 2.0, 3.0 };
  private final double potGround = 0;
  private final double potHatch1 = 1;
  private final double potHatch2 = 2;
  private final double potHatch3 = 3;
  private final double potBall1 = 1;
  private final double potBall2 = 2;
  private final double potBall3 = 3;
  private boolean isBallMode = false;
  private boolean isAutoMode = false;
  private int elevatorPosition = 0;
  private static int lastDirection;

  public void setUpPIDController() {
    controller = new PIDController(kP, kI, kD, kF, Robot.ahrs, this);
    controller.setInputRange(0, 5.0);
    controller.setOutputRange(-0.5, 0.5);
    controller.setAbsoluteTolerance(kToleranceVolts);
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

  public Elevator() {
    LiveWindow.add(elevatorPot);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new cmdElevatorXbox());
  }

  public void elevatorXbox() {
    double down = Robot.oi.xboxDriver2.getRawAxis(2);
    double up = Robot.oi.xboxDriver2.getRawAxis(3);
    if (up > 0.02 || down > 0.02) {
      isAutoMode = false;
    }

    if (isAutoMode) {
      autoMode();
    } else { // in manual mode
      if (up > 0.02) {
        Robot.elevator.elevatorUp(up);
      } else if (down > 0.02) {
        Robot.elevator.elevatorDown(down);
      } else {
        Robot.elevator.elevatorStop();
      }
    }

  }

  public void autoMode() { // make elevator motors go to position
    // if (elevatorPosition == 0) {
    // // Drive elevator to pot ground
    // moveElevator(potGround);
    // } else if (elevatorPosition == 1) {
    // if (isBallMode == false) {
    // // drive elevator to pot position HATCH
    // moveElevator(potHatch1);
    // } else {
    // // drive elevator to pot position BALL
    // moveElevator(potBall1);
    // }
    // } else if (elevatorPosition == 2) {
    // if (isBallMode == false) {
    // // Hatch positionn 2
    // moveElevator(potHatch2);
    // } else {
    // // Ball position 2
    // moveElevator(potBall2);
    // }
    // } else {
    // if (isBallMode == false) {
    // // Hatch posstion 3
    // moveElevator(potHatch3);
    // } else {
    // // ball Postiition 3
    // moveElevator(potBall3);
    // }
    // }
    if (controller.onTarget()) {
      elevatorStop();
      isAutoMode = false;
    } else {
      elevatorMotor.set(elevatorSpeed);
    }

  }

  public void moveElevator(double target) {
    int direction;
    double currentPosition = elevatorPot.getVoltage();

    if (target > currentPosition) {
      direction = 1;
    } else {
      direction = -1;
    }
    if (direction != lastDirection && lastDirection != 0) {
      elevatorStop();
      isAutoMode = false;
    } else {
      elevatorMotor.set(0.5 * direction);
    }
    lastDirection = direction;
  }

  public void elevatorUp(double speed) {
    elevatorMotor.set(speed);
  }

  public void elevatorDown(double speed) {
    elevatorMotor.set(-speed);
  }

  public void elevatorStop() {
    elevatorMotor.stopMotor();
  }

  public void elevatorAutoUp() {
    isAutoMode = true;
    lastDirection = 0;
    if (elevatorPosition < 3) {
      elevatorPosition++;
      if (isBallMode) {
        enablePIDController(potBall[elevatorPosition]);
      } else {
        enablePIDController(potHatch[elevatorPosition]);
      }
    }
    SmartDashboard.putNumber("elevatorPosition", elevatorPosition);
  }

  public void elevatorAutoDown() {
    isAutoMode = true;
    lastDirection = 0;
    if (elevatorPosition > 0) {
      elevatorPosition--;
      if (isBallMode) {
        enablePIDController(potBall[elevatorPosition]);
      } else {
        enablePIDController(potHatch[elevatorPosition]);
      }
    }
    SmartDashboard.putNumber("elevatorPosition", elevatorPosition);
  }

  public void elevatorAutoReset() {
    elevatorPosition = 0;
    SmartDashboard.putNumber("elevatorPosition", elevatorPosition);
  }

  public void setBallMode() {
    isBallMode = true;
    // cgBallRumble cmd = new cgBallRumble();
    // cmd.start();
  }

  public void setHatchMode() {
    isBallMode = false;
    // cgBallRumble cmd = new cgBallRumble();
    // cmd.start();
  }

  public boolean isBallMode() {
    return isBallMode;
  }

  @Override
  public void pidWrite(double output) {
    elevatorSpeed = output;
  }

}
