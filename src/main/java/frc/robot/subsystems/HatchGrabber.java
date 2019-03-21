/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.cmdHatchGrabberXbox;

/**
 * Add your docs here.
 */
public class HatchGrabber extends Subsystem {
  public DoubleSolenoid Grabber = new DoubleSolenoid(RobotMap.hatchGrabberModuleID, RobotMap.grabberExtendID,
      RobotMap.grabberRetractID);
  public DoubleSolenoid HatchSlide = new DoubleSolenoid(RobotMap.hatchGrabberModuleID, RobotMap.hatchSlideExtend,
      RobotMap.hatchSlideRetract);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new cmdHatchGrabberXbox());
  }

  public void slideOut() {
    Grabber.set(Value.kForward);
  }

  public void slideIn() {
    Grabber.set(Value.kReverse);
  }

  public void grab() {
    Grabber.set(Value.kForward);
  }

  public void release() {
    Grabber.set(Value.kReverse);
  }
}
