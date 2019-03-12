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
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.cgAutoRocket;
import frc.robot.commands.cmdAutoDriveForward;
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
  NetworkTableEntry nteRangeInFront;
  NetworkTableEntry ntePotValue;
  Command autoCommand;

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

    compressor = new Compressor(1);
    compressor.setClosedLoopControl(true);
    LiveWindow.add(compressor);
    LiveWindow.add(elbow.armSolenoid);
    LiveWindow.add(wrist.armSolenoid);
    // LiveWindow.addActuator(wrist.hatchSolenoid);

    rumble = new Rumble();

    oi = new OI(tableInstance);
    SmartDashboard.putData(new cmdWristUp());
    SmartDashboard.putData(new cmdWristDown());
    SmartDashboard.putData(new cmdElbowDown());
    SmartDashboard.putData(new cmdElbowUp());

    nteRangeInFront = Shuffleboard.getTab("Setup").add("RangeInFront", Robot.driveSystem.rangeInFront.getRangeInches())
        .getEntry();
    Shuffleboard.getTab("Setup").add("RangeInBack", Robot.driveSystem.rangeInBack.getRangeInches());
    Shuffleboard.getTab("Setup").add("Camera Pitch", Robot.vision.pitch);
    Shuffleboard.getTab("Setup").add("Camera Yaw", Robot.vision.yaw);
    Shuffleboard.getTab("Setup").add("YAw", ahrs.getYaw());
    Shuffleboard.getTab("Setup").add("Pitch", ahrs.getPitch());
    Shuffleboard.getTab("Setup").add("Roll", ahrs.getRoll());
    Shuffleboard.getTab("Setup").add("visionAngle", Robot.driveSystem.visionAngle);
    ntePotValue = Shuffleboard.getTab("Setup").add("Pot Value", Robot.elevator.elevatorPot.getAverageVoltage())
        .getEntry();
    SmartDashboard.putBoolean("IsBallMode", Robot.elevator.isBallMode());
    Shuffleboard.getTab("Setup").add("frontRight", Robot.driveSystem.frontRightMotor.getOutputCurrent());
    Shuffleboard.getTab("Setup").add("frontLeft", Robot.driveSystem.frontLeftMotor.getOutputCurrent());
    Shuffleboard.getTab("Setup").add("backRight", Robot.driveSystem.backRightMotor.getOutputCurrent());
    Shuffleboard.getTab("Setup").add("backLeft", Robot.driveSystem.backRightMotor.getOutputCurrent());
    Shuffleboard.getTab("Setup").add("ClimberCurrent", Robot.climber.climberMotor.getOutputCurrent());

  }

  @Override
  public void robotPeriodic() {
    // Shuffleboard.getTab("Setup").add("RangeFinder",
    // Robot.wrist.rangeFinder.getAverageVoltage());
    // SmartDashboard.putNumber("RangeFinder",
    // Robot.wrist.rangeFinder.getAverageVoltage());
    // Shuffleboard.getTab("Setup").add("RangeToFloor",
    // Robot.climber.rangeToFloor.getRangeInches());

    nteRangeInFront.setDouble(Robot.driveSystem.rangeInFront.getRangeInches());
    ntePotValue.setDouble(Robot.elevator.elevatorPot.getAverageVoltage());

    double v = Robot.vision.getAngle();
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
    autoCommand = new cgAutoRocket();
    autoCommand.start();

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
