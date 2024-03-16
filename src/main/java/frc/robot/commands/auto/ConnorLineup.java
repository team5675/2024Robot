package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

// This command aligns the robot based on feedback from the limelight camera
public class ConnorLineup extends CommandBase {

    private static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static final NetworkTableEntry horizontalOffset = limelightTable.getEntry("tx");
    private static final Swerve drive = Swerve.getInstance();

    private static final Translation2d rightMovement = new Translation2d(0.0, -0.5);
    private static final Translation2d leftMovement = new Translation2d(0.0, 0.5);

    private static final double OFFSET_THRESHOLD = 0.01;
    private static final int MAX_ITERATIONS = 1000;
    private int iterations = 0;

    @Override
    public void initialize() {
        // Nothing to initialize
    }

    @Override
    public void execute() {
        // If there's a target and the horizontal offset is significant, adjust the alignment
        if (limelightTable.getEntry("tv").getDouble(0) != 0) {
            double offset = horizontalOffset.getDouble(0);
            if (Math.abs(offset) > OFFSET_THRESHOLD && iterations < MAX_ITERATIONS) {
                System.out.println("Offset: " + offset);
                Translation2d movement = (offset < 0) ? rightMovement : leftMovement;
                System.out.println("Moving " + ((offset < 0) ? "right" : "left"));
                drive.drive(movement, 0.0, false);
                iterations++;
            } else {
                if (iterations >= MAX_ITERATIONS) {
                    System.out.println("Max iterations reached. Alignment might not be possible.");
                } else {
                    System.out.println("Alignment completed.");
                }
            }
        } else {
            System.out.println("No target detected.");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Command ends, stop the movement
        // You may need to add additional logic here based on your requirements
       
    }
}

