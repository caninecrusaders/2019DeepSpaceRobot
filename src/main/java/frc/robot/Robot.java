/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.cgAutoRocketLeft;
import frc.robot.commands.cgAutoRocketLeftBack;
import frc.robot.commands.cgAutoRocketLeftBack2;
import frc.robot.commands.cgAutoRocketRight;
import frc.robot.commands.cgAutoRocketRightBack;
import frc.robot.commands.cgAutoShipMiddle;
import frc.robot.commands.cmdAutoDriveForward;
import frc.robot.commands.cmdAutoNothing;
import frc.robot.commands.cmdClimberExtend;
import frc.robot.commands.cmdClimberTrigger;
import frc.robot.commands.cmdElbowDown;
import frc.robot.commands.cmdElbowUp;
import frc.robot.commands.cmdWristDown;
import frc.robot.commands.cmdWristUp;
import frc.robot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;
  public static driveSystem driveSystem;
  public static AHRS ahrs;
  public static NetworkTableInstance tableInstance;
  public static Vision vision;
  public static Elevator elevator;
  public static Wrist wrist;
  public static Elbow elbow;
  public static Climber climber;
  public static Compressor compressor;
  public static Rumble rumble;
  public static Preferences prefs;
  public static HatchGrabber hatch;

  Command autoCommand;
  SendableChooser<Command> chooser = new SendableChooser<Command>();

  NetworkTableEntry nteRangeInFront;
  NetworkTableEntry ntePotValue;
  NetworkTableEntry nteCameraYaw;
  NetworkTableEntry nteCameraPitch;
  NetworkTableEntry nteVisionAngle;
  NetworkTableEntry nteYaw;
  NetworkTableEntry nteRangeInBack;
  NetworkTableEntry ntePitch;
  NetworkTableEntry nteRoll;
  NetworkTableEntry nteFrontRight;
  NetworkTableEntry nteFrontLeft;
  NetworkTableEntry nteBackRight;
  NetworkTableEntry nteBackLeft;
  NetworkTableEntry nteClimberCurrent;
  NetworkTableEntry nteRangeToFloor;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    RobotMap.init();
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
      // ahrs = new AHRS(SerialPort.Port.kUSB);
      // ahrs = new AHRS(I2C.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP" + ex.getMessage(), true);
    }
    ahrs.reset();
    driveSystem = new driveSystem();
    // LiveWindow.add(driveSystem);

    CameraServer.getInstance().startAutomaticCapture("cam0", 0);

    vision = new Vision();
    // visionTable.addEntryListener("VisionResults", vision,
    // EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);

    // SmartDashboard.putBoolean("optical", RobotMap.opticalFront.get());
    // CameraServer.getInstance().startAutomaticCapture();

    elevator = new Elevator();

    wrist = new Wrist();

    elbow = new Elbow();

    climber = new Climber();

    // hatch = new HatchGrabber();

    compressor = new Compressor(1);
    compressor.setClosedLoopControl(true);
    LiveWindow.add(compressor);
    LiveWindow.add(elbow.armSolenoid);
    LiveWindow.add(wrist.armSolenoid);
    // LiveWindow.addActuator(wrist.hatchSolenoid);

    rumble = new Rumble();

    oi = new OI(tableInstance);

    // On shuffleboard when first opened
    SmartDashboard.putData("Auto mode", chooser);
    chooser.setDefaultOption("Rocket left -1", new cgAutoRocketLeft(0.8));
    chooser.addOption("Rocket Left -2", new cgAutoRocketLeft(1.6));
    chooser.addOption("Rocket Right -1", new cgAutoRocketRight(0.8));
    chooser.addOption("Rocket Right -2", new cgAutoRocketRight(1.6));
    chooser.addOption("Ship Middle -1", new cgAutoShipMiddle());
    chooser.addOption("Do Nothing", new cmdAutoNothing());
    chooser.addOption("Rocket Left Back -1", new cgAutoRocketLeftBack(2.7));
    chooser.addOption("Rocket Right Back -1", new cgAutoRocketRightBack(2.7));
    chooser.addOption("Rocket Left Back -2", new cgAutoRocketLeftBack2(2.4));

    SmartDashboard.putData(new cmdWristUp());
    SmartDashboard.putData(new cmdWristDown());
    SmartDashboard.putData(new cmdElbowDown());
    SmartDashboard.putData(new cmdElbowUp());
    SmartDashboard.putData(new cmdClimberTrigger());
    SmartDashboard.putData(new cmdClimberExtend());

    // On the tab setup
    nteRangeInFront = Shuffleboard.getTab("Setup").add("RangeInFront", Robot.driveSystem.rangeInFront.getRangeInches())
        .getEntry();
    nteRangeInBack = Shuffleboard.getTab("Setup").add("RangeInBack", Robot.driveSystem.rangeInBack.getRangeInches())
        .getEntry();
    nteCameraPitch = Shuffleboard.getTab("Setup").add("Camera Pitch", Robot.vision.pitch).getEntry();
    nteCameraYaw = Shuffleboard.getTab("Setup").add("Camera Yaw", Robot.vision.yaw).getEntry();
    nteYaw = Shuffleboard.getTab("Setup").add("Yaw", ahrs.getYaw()).getEntry();
    ntePitch = Shuffleboard.getTab("Setup").add("Pitch", ahrs.getPitch()).getEntry();
    nteRoll = Shuffleboard.getTab("Setup").add("Roll", ahrs.getRoll()).getEntry();
    nteVisionAngle = Shuffleboard.getTab("Setup").add("Vision Angle", Robot.driveSystem.visionAngle).getEntry();
    ntePotValue = Shuffleboard.getTab("Setup").add("Pot Value", Robot.elevator.elevatorPot.getAverageVoltage())
        .getEntry();
    nteFrontRight = Shuffleboard.getTab("Setup").add("frontRight", Robot.driveSystem.frontRightMotor.getOutputCurrent())
        .getEntry();
    nteFrontLeft = Shuffleboard.getTab("Setup").add("frontLeft", Robot.driveSystem.frontLeftMotor.getOutputCurrent())
        .getEntry();
    nteBackRight = Shuffleboard.getTab("Setup").add("backRight", Robot.driveSystem.backRightMotor.getOutputCurrent())
        .getEntry();
    nteBackLeft = Shuffleboard.getTab("Setup").add("backLeft", Robot.driveSystem.backRightMotor.getOutputCurrent())
        .getEntry();
    nteClimberCurrent = Shuffleboard.getTab("Setup")
        .add("ClimberCurrent", Robot.climber.climberMotor.getOutputCurrent()).getEntry();
    nteRangeToFloor = Shuffleboard.getTab("Setup").add("RangeToFloor", Robot.climber.rangeToFloor.getRangeInches())
        .getEntry();

  }

  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Elevator Position", Robot.elevator.elevatorPosition);
    SmartDashboard.putBoolean("IsBallMode", Robot.elevator.isBallMode());
    nteRangeInFront.setDouble(Robot.driveSystem.rangeInFront.getRangeInches());
    ntePotValue.setDouble(Robot.elevator.elevatorPot.getAverageVoltage());
    nteCameraPitch.setDouble(Robot.vision.pitch);
    nteCameraYaw.setDouble(Robot.vision.yaw);
    nteVisionAngle.setDouble(Robot.driveSystem.visionAngle);
    nteYaw.setDouble(ahrs.getYaw());
    nteRangeInBack.setDouble(Robot.driveSystem.rangeInBack.getRangeInches());
    ntePitch.setDouble(ahrs.getPitch());
    nteRoll.setDouble(ahrs.getRoll());
    nteBackLeft.setDouble(Robot.driveSystem.backLeftMotor.getOutputCurrent());
    nteBackRight.setDouble(Robot.driveSystem.backRightMotor.getOutputCurrent());
    nteFrontLeft.setDouble(Robot.driveSystem.frontLeftMotor.getOutputCurrent());
    nteFrontRight.setDouble(Robot.driveSystem.frontRightMotor.getOutputCurrent());
    nteRangeToFloor.setDouble(Robot.climber.rangeToFloor.getRangeInches());

    // double v = Robot.vision.getAngle();
    // SmartDashboard.putBoolean("optical", RobotMap.opticalFront.get());
  }

  @Override
  public void disabledInit() {
    prefs = Preferences.getInstance();
    if (!prefs.containsKey("Pot Calibration")) {
      prefs.putDouble("Pot Calibration", Robot.elevator.getPotCalibration());
    }
    Robot.elevator.setPotCalibration(prefs.getDouble("Pot Calibration", Robot.elevator.getPotCalibration()));
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    // SmartDashboard.putBoolean("optical", RobotMap.opticalFront.get());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    autoCommand = chooser.getSelected();
    autoCommand.start();
    // autoCommand = new cgAutoRocketLeft();
    // autoCommand.start();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    // SmartDashboard.putBoolean("optical", RobotMap.opticalFront.get());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
