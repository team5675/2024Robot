package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class Limelight {
    

    public static Limelight instance;

    ShuffleboardTab limelightTab;

    Optional<Pose2d> limelightPose2d;
    Optional<Pose3d> limelightPose3d;
    
    DoubleArraySubscriber botPoseSub;

    NetworkTable limelightTable;
    NetworkTableEntry robotPose;

    double[] robotPoseDouble;

    double visionTimestamp;


    public Limelight() {

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        botPoseSub = limelightTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[]{});
        
        limelightPose2d = Optional.empty();
        limelightPose3d = Optional.empty();
    }

    public void periodic() {

        robotPoseDouble = botPoseSub.getAtomic(new double[] {}).value;

        //26 ft 3.5in
        //54 ft 3.25in;
        if (robotPoseDouble.length > 0) {
            limelightPose2d = Optional.of(new Pose2d(robotPoseDouble[0], robotPoseDouble[1], 
                Rotation2d.fromDegrees(robotPoseDouble[5])));

            limelightPose3d = Optional.of(new Pose3d(robotPoseDouble[0], robotPoseDouble[1], robotPoseDouble[2], 
                new Rotation3d(robotPoseDouble[3], robotPoseDouble[4], robotPoseDouble[5])));

            visionTimestamp = Timer.getFPGATimestamp() - (robotPoseDouble[6] / 1000);
            
        } else {

            limelightPose2d = Optional.empty();
            limelightPose3d = Optional.empty();
        }
    }


    /**
     * Get the lastest pose data field-to-robot from limelight
     * @return
     */
    public Optional<Pose2d> getPose2dData() {
        return limelightPose2d;
    }

    public Optional<Pose3d> getPose3dData() {
        return limelightPose3d;
    }

    /**
     * Transforms RobotPose by SpeakerPose to get relative pose of robot to speaker
     * @return Center of Robot to Center of Speaker as a Transform3d
     */
    public Transform3d getPoseRobotToSpeaker() {
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose()), 
            Constants.FieldConstants.getAllianceBasedSpeakerPose());
    }

    /**
     * Transforms RobotPose by AmpPose to get relative pose of robot to amp
     * @return Center of Robot to Center of Amp as a Transform3d
     */
    public Transform3d getPoseRobotToAmp() {
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose()), 
            Constants.FieldConstants.getAllianceBasedAmpPose());
    }

    /**
     * Transforms RobotPose by TrapPose to get relative pose of robot to trap
     * @return Center of Robot to Center of Trap as a Transform3d
     */
    public Transform3d getPoseRobotToTrap() {
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose()),
            Constants.FieldConstants.getAllianceBasedTrapPose());
    }


    public Transform3d getPoseLauncherToTrap() {
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose())
            .plus(Constants.LauncherConstants.launcherMouthLocationXYZ),
            Constants.FieldConstants.getAllianceBasedTrapPose());
    }

    public Transform3d getPoseLauncherToSpeaker() {
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose())
            .plus(Constants.LauncherConstants.launcherMouthLocationXYZ),
            Constants.FieldConstants.getAllianceBasedSpeakerPose());
    }

    public Transform3d getPoseLauncherToAmp() {
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose())
            .plus(Constants.LauncherConstants.launcherMouthLocationXYZ), 
            Constants.FieldConstants.getAllianceBasedAmpPose());
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
