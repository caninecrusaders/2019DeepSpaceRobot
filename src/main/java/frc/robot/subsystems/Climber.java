/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.cmdClimberXbox;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX climberMotor = new WPI_TalonSRX(RobotMap.climberMotorID);
  public WPI_TalonSRX climberMotor2 = new WPI_TalonSRX(RobotMap.climberMotor2ID);
  public Ultrasonic rangeToFloor = new Ultrasonic(RobotMap.floorTriggerID, RobotMap.floorEchoID);
  public DoubleSolenoid climberTrigger = new DoubleSolenoid(RobotMap.climberModuleID, RobotMap.climberFootExtendID,
      RobotMap.climberFootRetractID);
  public double timeExtend = 0;
  public double startingPitch = 0;

  public Climber() {
    climberMotor.configOpenloopRamp(1.0);
    climberMotor2.configOpenloopRamp(1.0);
    climberTrigger.set(Value.kReverse);
  }

  public void triggerFoot() {
    climberTrigger.set(Value.kForward);
  }

  public void triggerReset() {
    climberTrigger.set(Value.kReverse);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    rangeToFloor.setAutomaticMode(true);
    setDefaultCommand(new cmdClimberXbox());
  }

  public void up() {
    climberMotor.set(-0.5);
    climberMotor2.set(-0.5);
  }

  public void up(double speed) {
    climberMotor.set(-speed);
    climberMotor2.set(-speed);
  }

  public void down() {
    climberMotor.set(1.0);
    climberMotor2.set(1.0);
  }

  public void stop() {
    climberMotor.stopMotor();
    climberMotor2.stopMotor();
  }

}
