package frc.robot.subsystems;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.cmdVisionXbox;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Vision extends Subsystem implements TableEntryListener {
    int[] filters;
    int index = 0;
    public NetworkTable visionTable;
    public NetworkTableInstance tableInstance;
    public Servo CameraYaw = new Servo(0);
    public Servo CameraPitch = new Servo(1);
    public double yaw = 0.27;
    public double pitch = 0.83;
    // public NetworkTable filterTable;
    double h;
    double s;
    double v;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public Vision() {
        // SmartDashboard.putString("VisionAngle", "waiting");

        tableInstance = NetworkTableInstance.getDefault();
        // tableInstance.startClientTeam(4980);
        // tableInstance.startClient("10.49.80.2", 1735);

        visionTable = tableInstance.getTable("CVResultsTable");
        // visionTable.addEntryListener("VisionResults", this,
        // EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        // visionTable.addEntryListener("VisionResults", (table, key, entry, value,
        // flags) -> {
        // String[] values = value.getString().split(",");
        // if (Integer.parseInt(values[0]) == -1) {
        // // SmartDashboard.putString("VisionAngle", "invalid");
        // Robot.driveSystem.isVisionAngleValid = false;
        // return;
        // }

        // Robot.driveSystem.visionAngle = Double.parseDouble(values[1]);
        // Robot.driveSystem.isVisionAngleValid = true;
        // // SmartDashboard.putString("VisionAngle", values[1]);

        // }, EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        // // visionTable.addEntryListener("VisionResults", OI::VisionResultsChanged,
        // // EntryListenerFlags.kUpdate);
        // // filterTable = tableInstance.getTable("hsvFilter");
        // // filters = new int[]{0,27,33,25,255,15,255};
        // // filterTable.getEntry("minH").setNumber(filters[1]);
        // // filterTable.getEntry("maxH").setNumber(filters[2]);
        // // filterTable.getEntry("minS").setNumber(filters[3]);
        // filterTable.getEntry("maxS").setNumber(filters[4]);
        // filterTable.getEntry("minV").setNumber(filters[5]);
        // filterTable.getEntry("maxV").setNumber(filters[6]);

    }

    public double getAngle() {

        // visionTable.addEntryListener("VisionResults", this,
        // EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);

        String[] values = visionTable.getEntry("VisionResults").getString("-1,-1").split(",");
        if (Integer.parseInt(values[0]) == 0) {
            SmartDashboard.putString("VisionAngle", "invalid");
            Robot.driveSystem.isVisionAngleValid = false;
        }

        Robot.driveSystem.visionAngle = Double.parseDouble(values[1]);
        Robot.driveSystem.isVisionAngleValid = true;
        return Robot.driveSystem.visionAngle;

    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new cmdVisionXbox());
    }

    public void incrementFilter() {
        // filters[index]++;
        setFilters();
    }

    public void decrementFilter() {
        // filters[index]--;
        setFilters();
    }

    public void incrementIndex() {
        // index = (index+1)%filters.length;
        setFilters();
    }

    public void setFilters() {
        // filters[1] = (int) SmartDashboard.getNumber("minH", filters[1]);
        // filters[2] = (int) SmartDashboard.getNumber("maxH", filters[2]);
        // filters[3] = (int) SmartDashboard.getNumber("minS", filters[3]);
        // filters[4] = (int) SmartDashboard.getNumber("maxS", filters[4]);
        // filters[5] = (int) SmartDashboard.getNumber("minV", filters[5]);
        // filters[6] = (int) SmartDashboard.getNumber("maxV", filters[6]);
        // filterTable.getEntry("minH").setNumber(filters[1]);
        // filterTable.getEntry("maxH").setNumber(filters[2]);
        // filterTable.getEntry("minS").setNumber(filters[3]);
        // filterTable.getEntry("maxS").setNumber(filters[4]);
        // filterTable.getEntry("minV").setNumber(filters[5]);
        // filterTable.getEntry("maxV").setNumber(filters[6]);
        // SmartDashboard.putNumber("minH", filters[1]);
        // SmartDashboard.putNumber("maxH", filters[2]);
        // SmartDashboard.putNumber("minS", filters[3]);
        // SmartDashboard.putNumber("maxS", filters[4]);
        // SmartDashboard.putNumber("minV", filters[5]);
        // SmartDashboard.putNumber("maxV", filters[6]);
        // }
        // other

    }

    public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
            int flags) {
        String[] values = value.getString().split(",");
        if (Integer.decode(values[0]) == -1) {
            SmartDashboard.putString("VisionAngle", "invalid");
            Robot.driveSystem.isVisionAngleValid = false;
            return;
        }

        Robot.driveSystem.visionAngle = Double.parseDouble(values[1]);
        Robot.driveSystem.isVisionAngleValid = true;
        SmartDashboard.putString("VisionAngle", values[1]);

    }

    public void visionControl() {
        // double ServoPitch = Robot.oi.xboxDriver.getRawAxis(5);
        // double ServoYaw = Robot.oi.xboxDriver.getRawAxis(0);

        // if (ServoYaw > 0.5) {
        // yaw += 0.01;
        // } else if (ServoYaw < -0.5) {
        // yaw -= 0.01;
        // }
        // if (ServoPitch > 0.5) {
        // pitch += 0.01;
        // } else if (ServoPitch < -0.5) {
        // pitch -= 0.01;
        // }
        CameraYaw.set(yaw);
        CameraPitch.set(pitch);
    }
}
