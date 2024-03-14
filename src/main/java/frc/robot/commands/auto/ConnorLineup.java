package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

// This command aligns the robot based on feedback from the limelight camera
public class ConnorLineup extends Command {

    static NetworkTable limelightTable;
    static NetworkTableEntry horizontalOffset;
    static NetworkTableEntry verticalOffset;
    static NetworkTableEntry verticalAngleOffset;
    static NetworkTableEntry isTarget;
    Swerve drive;

    @Override
    public void initialize() {
        // Initialize the drive subsystem
        drive = Swerve.getInstance();
    }

    @Override
    public void execute() {
        // Retrieve values from the limelight network table
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        horizontalOffset = limelightTable.getEntry("tx");
        verticalOffset = limelightTable.getEntry("ty");
        isTarget = limelightTable.getEntry("tv");
        // Define translation movements for aligning left and right
        Translation2d rightMovement = new Translation2d(0.0, -0.1);
        Translation2d leftMovement = new Translation2d(0.0, 0.1);

        // Loop until horizontal offset is close to 0
        while (Math.abs(horizontalOffset.getDouble(0)) > 0.01) {
            // If the offset is negative, move right
            if (horizontalOffset.getDouble(0) < 0) {
                drive.drive(rightMovement, 0.0, false);
            } else {
                // If the offset is positive, move left
                drive.drive(leftMovement, 0.0, false);
            }
            // Update horizontal offset value
            horizontalOffset = limelightTable.getEntry("tx");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Command ends, stop the movement
        // You may need to add additional logic here based on your requirements
    }
}


