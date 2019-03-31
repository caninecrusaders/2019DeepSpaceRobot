/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.cmdElbowXbox;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Add your docs here.
 */
public class Elbow extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DoubleSolenoid armSolenoid = new DoubleSolenoid(RobotMap.elbowModuleID, RobotMap.elbowDownID,
      RobotMap.elbowUpID);
  // private boolean ElbowDown = false;
  // private boolean ElbowUp = true;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new cmdElbowXbox());

  }

  public void ElbowUp() {
    armSolenoid.set(Value.kReverse);
  }

  public void ElbowDown() {
    armSolenoid.set(Value.kForward);
  }

  public void ElbowOff() {
    armSolenoid.set(Value.kOff);
  }
}
