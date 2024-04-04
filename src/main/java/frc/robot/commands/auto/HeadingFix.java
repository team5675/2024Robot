package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveDrive;

// This command aligns the robot based on feedback from the limelight camera
public class HeadingFix extends Command {

    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry horizontalOffset = limelightTable.getEntry("tx");
    private NetworkTableEntry verticalOffset = limelightTable.getEntry("ty");
    private Swerve drive = Swerve.getInstance();
    

   private double rotateX = Math.cos(Math.toRadians(30));
   private double rotateY = Math.sin(Math.toRadians(30));
    private Translation2d rightMovement = new Translation2d(0.0, -0.25);
    private Translation2d leftMovement = new Translation2d(0.0, 0.25);
   private Translation2d forwardMovement = new Translation2d(-0.25, 0);
   private Translation2d backMovement = new Translation2d(0.25, 0);
   //private Translation2d rotateClockwise = new Translation2d(rotateX , rotateY); // 30 degrees clockwise
   //private Translation2d rotateNotClockwise = new Translation2d(rotateX , -rotateY); // 30 degrees counterclockwise (or anticlockwise)
    private Translation2d noMove = new Translation2d(0,0);
    double rotateClockwise = 0.3;
    double rotateNotClockwise = -0.3;

    private double OFFSET_THRESHOLD = 0.5;
    private double vertOFFSET_THRESHOLD = 0.3;
   private double angOFFSET_THRESHOLD = 1.0;

    private double kLimeLightVerticalAngle = verticalOffset.getDouble(0);
    private double kTargetDistance = 1.2192;
     private double kLimeLightAngle = 45.0;
     private double kAprilTagHeight = 1.2192;
     private double kLimelightHeight = 0.6096;
     private double kLimelightDiff;
     private double kDistance;
     private double aprilTagID;
     private double aprilTagOffset = 0.0;
     private Rotation2d rawHeading = Swerve.getInstance().getGyroAngle();
    private double heading = rawHeading.getDegrees();
    private double angError = 0.0;
     
  

    int atagIDConvert(){
        aprilTagID = limelightTable.getEntry("tid").getDouble(-1);
        int aprilTagIDInt = (int) aprilTagID;
        return aprilTagIDInt;
    }

     public double aprilTagPlainHeading(int tagId) {
        switch (tagId) {
            case 11:
            return 120; // Heading angle for tag ID 11 Red Left Stage
        case 12:
            return -120; // Heading angle for tag ID 12 Red Right Stage
        case 13:
            return 0; // Heading angle for tag ID 13 Red Center Stage
        case 14:
            return 0; // Heading angle for tag ID 14 Blue Center Stage
        case 15:
            return 120; // Heading angle for tag ID 15 Blue Left Stage
        case 16:
            return -120; // Heading angle for tag ID 16 Blue Right Stage
        default:
            return -1; // Invalid tag ID, return -1 or throw an exceptio
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
        
         idealHeading = aprilTagPlainHeading(atagIDConvert());
         rawHeading = Swerve.getInstance().getGyroAngle();
                heading = rawHeading.getDegrees();
            System.out.println("April Tag ID:" + atagIDConvert());
            if (!MathUtil.isNear(idealHeading,heading,angOFFSET_THRESHOLD)){
                angError = (idealHeading - heading)/3.0;
                System.out.println("Updated Heading:" + heading);
                System.out.println("Ideal Heading" + idealHeading);
                double rotateClockwise = 0.5*angError;
                double rotateNotClockwise = 0.5*angError;
                //Translation2d rotate = (heading > idealHeading) ? rotateNotClockwise : rotateClockwise;
                double headingError = idealHeading - heading;
                double rotate = (headingError > 0) ? rotateNotClockwise : rotateClockwise;
                System.out.println(rotate/angError);
                drive.drive(noMove,rotate,true);
                rawHeading = Swerve.getInstance().getGyroAngle();
                heading = rawHeading.getDegrees();
       }
} else{
    System.out.println("No April Tage Detected");
}
}
    
    @Override
    public boolean isFinished() {
        
        //return when note in launcher
        return MathUtil.isNear(idealHeading,heading,angOFFSET_THRESHOLD);
    }
            

    @Override
    public void end(boolean interrupted) {
        // Command ends, stop the movement
        // You may need to add additional logic here based on your requirements
       
    }
}

