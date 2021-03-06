/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {

  // public static WPI_TalonSRX testMotor;

  // public static DigitalInput opticalFront;
  // public static DigitalInput opticalMiddle;
  public static final int wristModuleID = 1;
  public static final int wristUpID = 0;
  public static final int wristDownID = 1;

  public static final int elbowModuleID = 1;
  public static final int elbowUpID = 2;
  public static final int elbowDownID = 3;

  public static final int climberModuleID = 1;
  public static final int climberFootExtendID = 4;
  public static final int climberFootRetractID = 5;

  // Hatch Grabber
  public static final int hatchGrabberModuleID = 11;
  public static final int grabberExtendID = 14;
  public static final int grabberRetractID = 15;
  public static final int hatchSlideExtend = 16;
  public static final int hatchSlideRetract = 17;

  // Old Hatch Pickup
  public static final int hatchExtendID = 7;
  public static final int hatchRetractID = 6;
  public static final int hatchModuleID = 1;

  public static final int driveBackLeftMotorID = 2;
  public static final int driveBackRightMotorID = 3;
  public static final int driveFrontLeftMotorID = 4;
  public static final int driveFrontRightMotorID = 5;
  public static final int climberMotorID = 6;
  public static final int intakeMotorID = 7;
  public static final int elevatorMotorID = 8;
  public static final int climberMotor2ID = 9;

  public static final int elevatorPotID = 1;

  public static final int rangeFinderID = 0;

  public static final int backTriggerID = 4;
  public static final int backEchoID = 5;

  public static final int floorTriggerID = 6;
  public static final int floorEchoID = 7;

  public static final int frontEchoID = 8;
  public static final int frontTriggerID = 9;

  public static void init() {

    // testMotor = new WPI_TalonSRX(6);
    // testMotor.setInverted(false);

    // opticalFront = new DigitalInput(9);
    // opticalMiddle = new DigitalInput(8);

  }

}
