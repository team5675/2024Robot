package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Limelight {
    

    public static Limelight instance;

    ShuffleboardTab limelightTab;

    Pose2d limelightPose2d;
    DoubleArraySubscriber botPoseSub;

    NetworkTable limelightTable;
    NetworkTableEntry robotPose;

    double[] robotPoseDouble;

    double visionTimestamp;


    public Limelight() {

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        botPoseSub = limelightTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[]{});
        
        limelightPose2d = new Pose2d();
    }

    public void periodic() {

        robotPoseDouble = botPoseSub.getAtomic(new double[] {}).value;

        //26 ft 3.5in
        //54 ft 3.25in;
        if (robotPoseDouble.length > 0) {
            limelightPose2d = new Pose2d(robotPoseDouble[0], robotPoseDouble[1], Rotation2d.fromDegrees(robotPoseDouble[5]));
            visionTimestamp = Timer.getFPGATimestamp() - (robotPoseDouble[6] / 1000);
        }
    }


    /**
     * Get the lastest pose data field-to-robot from limelight
     * @return
     */
    public Pose2d getPoseData() {
        return limelightPose2d;
    }

    /**
     * Get timestamp of latest pose data
     * @return 
     */
    public double getTimestampData() {
        return visionTimestamp;
    }

    public static Limelight getInstance() {
        if(instance == null)
            instance = new Limelight();

        return instance;
    }
}
