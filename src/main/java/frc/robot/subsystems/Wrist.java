/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.cmdWristXbox;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Add your docs here.
 */
public class Wrist extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final DoubleSolenoid armSolenoid = new DoubleSolenoid(RobotMap.wristModuleID, RobotMap.wristDownID,
      RobotMap.wristUpID);
  private final DoubleSolenoid hatchSolenoid = new DoubleSolenoid(RobotMap.hatchModuleID, RobotMap.hatchExtendID,
      RobotMap.hatchRetractID);
  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(RobotMap.intakeMotorID);
  private boolean WristDown = false;
  private boolean WristUp = true;
  public final AnalogInput rangeFinder = new AnalogInput(RobotMap.rangeFinderID);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new cmdWristXbox());
  }

  public void wristUp() {
    armSolenoid.set(Value.kForward);
  }

  public void wristDown() {
    armSolenoid.set(Value.kReverse);
  }

  public void wristOff() {
    armSolenoid.set(Value.kOff);
  }

  public void intakeIn(double speed) {
    intakeMotor.set(speed);
  }

  public void intakeOut(double speed) {
    intakeMotor.set(-speed);
  }

  public void intakeStop() {
    intakeMotor.set(0);
  }

  public void hatchExtend() {
    hatchSolenoid.set(Value.kForward);
  }

  public void hatchRetract() {
    hatchSolenoid.set(Value.kReverse);
  }
}
