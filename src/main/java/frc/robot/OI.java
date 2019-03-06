/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.EntryListenerFlags;
import frc.robot.commands.*;

public class OI {
    public Joystick xboxDriver;
    public Joystick xboxDriver2;
    public JoystickButton leftBumper;
    public JoystickButton rightBumper;
    public JoystickButton xButton;
    public JoystickButton bButton;
    public JoystickButton resetButton;
    public JoystickButton hatchButton2;
    public JoystickButton ballButton2;
    public NetworkTable visionTable;
    public JoystickButton leftBumper2;
    public JoystickButton rightBumper2;
    public JoystickButton xButton2;
    public JoystickButton bButton2;
    public JoystickButton aButton2;
    public JoystickButton yButton;
    public JoystickButton yButton2;
    public JoystickButton aButton;
    public JoystickButton resetButton2;

    public OI(NetworkTableInstance tableInstance) {
        xboxDriver = new Joystick(0);

        xboxDriver2 = new Joystick(1);

        // controller 1
        rightBumper = new JoystickButton(xboxDriver, 6);
        rightBumper.whileHeld(new cmdClimberLegOut());

        leftBumper = new JoystickButton(xboxDriver, 5);
        leftBumper.whileHeld(new cmdClimberLegIn());

        resetButton = new JoystickButton(xboxDriver, 8);
        resetButton.whenPressed(new cmdResetGyro());

        yButton = new JoystickButton(xboxDriver, 4);
        yButton.whenPressed(new cgClimber());

        aButton = new JoystickButton(xboxDriver, 1);
        aButton.whileHeld(new cmdVisionMode());

        // controller 2
        rightBumper2 = new JoystickButton(xboxDriver2, 6);
        rightBumper2.whenPressed(new cmdAutoUp());

        leftBumper2 = new JoystickButton(xboxDriver2, 5);
        leftBumper2.whenPressed(new cmdAutoDown());

        bButton2 = new JoystickButton(xboxDriver2, 2);
        bButton2.whenPressed(new cgIntakeBall());

        xButton2 = new JoystickButton(xboxDriver2, 3);
        xButton2.whenPressed(new cmdResetElevator());

        aButton2 = new JoystickButton(xboxDriver2, 1);
        aButton2.whenPressed(new cgIntakeHatch());

        yButton2 = new JoystickButton(xboxDriver2, 4);
        yButton2.whenPressed(new cgStartPosition());

        ballButton2 = new JoystickButton(xboxDriver2, 8);
        ballButton2.whenPressed(new cmdBallMode());

        hatchButton2 = new JoystickButton(xboxDriver2, 7);
        hatchButton2.whenPressed(new cmdHatchMode());

        // // other
        // visionTable = tableInstance.getTable("CVResultsTable");
        // visionTable.addEntryListener("VisionResults", OI::VisionResultsChanged,
        // EntryListenerFlags.kUpdate & EntryListenerFlags.kNew);
        // SmartDashboard.putString("VisionAngle", "waiting");
        // }
    }

    // public static void VisionResultsChanged(NetworkTable table, String key,
    // NetworkTableEntry entry,
    // NetworkTableValue value, int flags) {
    // String[] values = value.getString().split(",");
    // // if (Integer.decode(values[0]) == -1) {
    // // SmartDashboard.putString("VisionAngle", "invalid");
    // // Robot.driveSystem.isVisionAngleValid = false;
    // // return;
    // // }

    // Robot.driveSystem.visionAngle = Double.parseDouble(values[1]);
    // Robot.driveSystem.isVisionAngleValid = true;
    // SmartDashboard.putString("VisionAngle", values[1]);

    // }
}
