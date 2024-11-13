package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;

    public LimelightSubsystem() {
        
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        
  // Assuming the Limelight camera is configured correctly
    }

    // Get the horizontal offset (degrees) to the target
    public double getTX() {
        return limelightTable.getEntry("tx").getDouble(0.0); // Default 0.0 if no target detected
    }
    public double getTA() {
        return limelightTable.getEntry("ta").getDouble(0.0); // Default 0.0 if no target detected
    }

    // Optionally, you can add other methods to get more data from the Limelight
    // like target distance (ty), target area (ta), etc.
}

