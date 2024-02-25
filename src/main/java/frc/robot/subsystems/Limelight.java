package frc.robot.subsystems;

import java.util.Optional;

//import org.photonvision.PhotonCamera;

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
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

public class Limelight {
    

    public static Limelight instance;

    ShuffleboardTab limelightTab;
    
    DoubleArraySubscriber botPoseSub;

    NetworkTable limelightTable;
    NetworkTableEntry robotPose;

    double[] robotPoseDouble;

    double visionTimestamp;

    public class PosePacket {
        Optional<Pose2d> pose2d;
        Optional<Pose3d> pose3d;
        Optional<Double> timestamp;
    }

    PosePacket posePacket;

    public Limelight() {

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        botPoseSub = limelightTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[]{});
        
        posePacket = new PosePacket();
        posePacket.pose2d = Optional.empty();
        posePacket.pose3d = Optional.empty();
        posePacket.timestamp = Optional.empty();
    }

    public void periodic() {

        robotPoseDouble = botPoseSub.getAtomic(new double[] {}).value;

        //26 ft 3.5in
        //54 ft 3.25in;
        if (robotPoseDouble.length > 0) {
            posePacket.pose2d = Optional.of( new Pose2d(robotPoseDouble[0], robotPoseDouble[1], 
                Rotation2d.fromDegrees(robotPoseDouble[5])));

            posePacket.pose3d = Optional.of(new Pose3d(robotPoseDouble[0], robotPoseDouble[1], robotPoseDouble[2], 
                new Rotation3d(robotPoseDouble[3], robotPoseDouble[4], robotPoseDouble[5])));

            posePacket.timestamp = Optional.of(Timer.getFPGATimestamp() - (robotPoseDouble[6] / 1000));
            
        } else {

            posePacket.pose2d = Optional.empty();
            posePacket.pose3d = Optional.empty();
            posePacket.timestamp = Optional.empty();
        }
    }


    /**
     * Get the lastest pose data field-to-robot from limelight
     * @return
     */
    public PosePacket getPose2dData() {
        return posePacket;
    }

    public PosePacket getPose3dData() {
        return posePacket;
    }

    /**
     * Transforms RobotPose by SpeakerPose to get relative pose of robot to speaker
     * @return Center of Robot to Center of Speaker as a Transform3d
     */
    public Transform3d getPoseRobotToSpeaker() {
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose()), 
            AllianceFlipUtil.apply(FieldConstants.Speaker.speakerCenterPose));
    }

    /**
     * Transforms RobotPose by AmpPose to get relative pose of robot to amp
     * @return Center of Robot to Center of Amp as a Transform3d
     */
    public Transform3d getPoseRobotToAmp() {
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose()), 
            AllianceFlipUtil.apply(FieldConstants.ampCenterPose));
    }

    /**
     * Transforms RobotPose by TrapPose to get relative pose of robot to trap
     * @return Center of Robot to Center of Trap as a Transform3d
     */
    public Transform3d getPoseRobotToTrap() {
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose()),
            AllianceFlipUtil.apply(FieldConstants.Stage.centerPose));
    }


    /**
     * Returns the relative distance (x,y,z) and angle (roll, pitch, yaw)
     * <p>of the current launcher mouth to the trap location
     * @return Launcher Mouth to Trap Vector
     */
    public Transform3d getPoseLauncherToTrap() {
        //get the origin to robot center vector
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose())
        //add the robot center to launcher mouth home vector
            .plus(Constants.LauncherConstants.launcherMouthHomeLocationXYZ)
        //add the current launcher mouth vector against the origin launcher mouth vector
            .plus(Wristavator.getInstance().getRobotBaseToLauncherMouthPose()),
        //finally place in origin to trap vector
            AllianceFlipUtil.apply(FieldConstants.Stage.centerPose));
    }

    public Transform3d getPoseLauncherToSpeaker() {

        //get the origin to robot center vector
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose())
        //add the robot center to launcher mouth home vector
            .plus(Constants.LauncherConstants.launcherMouthHomeLocationXYZ)
        //add the current launcher mouth vector against the origin launcher mouth vector
            .plus(Wristavator.getInstance().getRobotBaseToLauncherMouthPose()),
            AllianceFlipUtil.apply(FieldConstants.Speaker.speakerCenterPose));
    }

    public Transform3d getPoseLauncherToAmp() {
        //get the origin to robot center vector
        return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose())
        //add the robot center to launcher mouth home vector
            .plus(Constants.LauncherConstants.launcherMouthHomeLocationXYZ)
        //add the current launcher mouth vector against the origin launcher mouth vector
            .plus(Wristavator.getInstance().getRobotBaseToLauncherMouthPose()),
            AllianceFlipUtil.apply(FieldConstants.ampCenterPose));
    }
    

    public static Limelight getInstance() {
        if(instance == null)
            instance = new Limelight();

        return instance;
    }
}
