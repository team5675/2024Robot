package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveDrive;

// This command aligns the robot based on feedback from the limelight camera
public class ConnorLineup extends CommandBase {

    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry horizontalOffset = limelightTable.getEntry("tx");
    private NetworkTableEntry verticalOffset = limelightTable.getEntry("ty");
    private Swerve drive = Swerve.getInstance();
    SwerveDrive swerveDrive;

    private double rotateX = Math.cos(Math.toRadians(30));
     private double rotateY = Math.sin(Math.toRadians(30));
    private Translation2d rightMovement = new Translation2d(0.0, -0.2);
    private Translation2d leftMovement = new Translation2d(0.0, 0.2);
    private Translation2d forwardMovement = new Translation2d(-0.2, 0);
    private Translation2d backMovement = new Translation2d(0.2, 0);
    private Translation2d rotateClockwise = new Translation2d(rotateX , rotateY); // 30 degrees clockwise
    private Translation2d rotateNotClockwise = new Translation2d(rotateX , -rotateY); // 30 degrees counterclockwise (or anticlockwise)
    
    private double OFFSET_THRESHOLD = 0.01;
    private double vertOFFSET_THRESHOLD = 0.1;
    private double angOFFSET_THRESHOLD = 0.1;

    private double kLimeLightVerticalAngle = verticalOffset.getDouble(0);
    private double kTargetDistance = 48.0;
     private double kLimeLightAngle = 0;
     private double kAprilTagHeight = 2.032;
     private double kLimelightHeight = 0;
     private double kLimelightDiff = 0;
     private double kDistance = 0;
     private double aprilTagID = 0;
      private Rotation2d rawHeading = swerveDrive.getOdometryHeading();
     private double heading = rawHeading.getDegrees();
    
     
    

    int atagIDConvert(){
        aprilTagID = limelightTable.getEntry("tid").getDouble(-1);
        int aprilTagIDInt = (int) aprilTagID;
        return aprilTagIDInt;
    }

     public double aprilTagPlainHeading(int tagId) {
        switch (tagId) {
            case 13:
                return 0; // Heading angle for tag ID 1
            case 14:
                return 60; // Heading angle for tag ID 2
            case 15:
                return 120; // Heading angle for tag ID 3
            case 16:
                return 180; // Heading angle for tag ID 4
            case 17:
                return 240; // Heading angle for tag ID 5
            case 18:
                return 300; // Heading angle for tag ID 6
            default:
                return -1; // Invalid tag ID, return -1 or throw an exception
        }
    }

    private double idealHeading = aprilTagPlainHeading(atagIDConvert());

    @Override
    public void initialize() {
        // Nothing to initialize
    }

    @Override
    public void execute() {
        // If there's a target and the horizontal offset is significant, adjust the alignment
        if (limelightTable.getEntry("tv").getDouble(0) != 0) {
            aprilTagID = limelightTable.getEntry("tid").getDouble(-1);
            System.out.println("April Tag ID:" + atagIDConvert());
            while (Math.abs(idealHeading - heading) > angOFFSET_THRESHOLD){
                System.out.println("Updated Heading:" + heading);
                Translation2d rotate = (heading > idealHeading) ? rotateNotClockwise : rotateClockwise;
                drive.drive(rotate,0.0,false);
                heading = rawHeading.getDegrees();
            }

            double aprilTagOffset = horizontalOffset.getDouble(0);
            System.out.println("Offset: " + aprilTagOffset);
            while (Math.abs(aprilTagOffset) > OFFSET_THRESHOLD){
                System.out.println("Updated Offset:" + aprilTagOffset);
                Translation2d movement = (aprilTagOffset < 0) ? rightMovement : leftMovement;
                drive.drive(movement, 0.0, false);
                aprilTagOffset = horizontalOffset.getDouble(0);
            }
        
        while (Math.abs(kTargetDistance - kDistance) > vertOFFSET_THRESHOLD) {
            System.out.println("Distance:" + kDistance );
            Translation2d yAxisMovement = (kLimeLightVerticalAngle < 0) ? forwardMovement : backMovement;
            drive.drive(yAxisMovement, 0.0, false);
            kLimelightDiff = (kAprilTagHeight - kLimelightHeight);
            kDistance = kLimelightDiff/Math.tan(kLimeLightAngle + kLimeLightVerticalAngle);
            kLimeLightVerticalAngle = verticalOffset.getDouble(0);
        } 
    }   
    else {
            System.out.println("No April Tag Detected");
                }
            }
        
    
            
            

    @Override
    public void end(boolean interrupted) {
        // Command ends, stop the movement
        // You may need to add additional logic here based on your requirements
       
    }
}

