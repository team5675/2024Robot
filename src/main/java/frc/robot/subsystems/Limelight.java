package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

//import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

public class Limelight extends SubsystemBase{
    

    public static Limelight instance;

    ShuffleboardTab limelightTab;
    
    DoubleArraySubscriber botPoseSub;

    NetworkTable limelightTable;
    NetworkTableEntry robotPose;

    double[] robotPoseDouble;

    double visionTimestamp;

    BooleanSupplier atPoseSupplier;
    Trigger atPoseTrigger;

    Pose2d desiredPose;
    Pose2d currentPose;

    double aprilTagID;
    boolean isValidTagTarget;

    PIDController xPID;
    PIDController yPID;
    PIDController omegaPID;

    public enum TargetID {
        AMP,
        TRAP,
    }

    TargetID targetID;

    public class PosePacket {
        Optional<Pose2d> pose2d;
        Optional<Pose3d> pose3d;
        Optional<Double> timestamp;
    }

    PosePacket posePacket;

     public Limelight() {}}

//         limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
//         botPoseSub = limelightTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[]{});
        
//         posePacket = new PosePacket();
//         posePacket.pose2d = Optional.empty();
//         posePacket.pose3d = Optional.empty();
//         posePacket.timestamp = Optional.empty();

//         desiredPose = new Pose2d();
//         currentPose = new Pose2d();

//         aprilTagID = -1;
//         targetID = TargetID.AMP;

//         xPID = new PIDController(0.5, 0, 0);
//         yPID = new PIDController(0.5, 0, 0);
//         omegaPID = new PIDController(0.05, 0, 0.001);

//         omegaPID.enableContinuousInput(-180, 180);

//         atPoseSupplier = new BooleanSupplier() {
//             @Override
//             public boolean getAsBoolean() {
//                 return  xPID.atSetpoint() && yPID.atSetpoint() && omegaPID.atSetpoint();
//             }
//         };
//         atPoseTrigger = new Trigger(atPoseSupplier);
//     }

//     public Trigger getAtPose() {
//         return atPoseTrigger;
//     }

//     public void setTargetID(TargetID id) {
//         targetID = id;
//     }

//     @Override
//     public void periodic() {

//         robotPoseDouble = botPoseSub.getAtomic(new double[] {}).value;

//         //26 ft 3.5in
//         //54 ft 3.25in;
//         if (robotPoseDouble.length > 0) {
//             posePacket.pose2d = Optional.of( new Pose2d(robotPoseDouble[0], robotPoseDouble[1], 
//                 Rotation2d.fromDegrees(robotPoseDouble[5])));

//             posePacket.pose3d = Optional.of(new Pose3d(robotPoseDouble[0], robotPoseDouble[1], robotPoseDouble[2], 
//                 new Rotation3d(robotPoseDouble[3], robotPoseDouble[4], robotPoseDouble[5])));

//             posePacket.timestamp = Optional.of(Timer.getFPGATimestamp() - (robotPoseDouble[6] / 1000));

//             aprilTagID = limelightTable.getEntry("tid").getDouble(-1);
            
//         } else {


//             posePacket.pose2d = Optional.empty();
//             posePacket.pose3d = Optional.empty();
//             posePacket.timestamp = Optional.empty();

//             aprilTagID = -1;
//         }

//         if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
//             if (targetID == TargetID.AMP && aprilTagID == 6)
//                 desiredPose = Constants.LimelightConstants.ampBlueShotLocation;
//             else {
//                 if(aprilTagID == 14)
//                     desiredPose = Constants.LimelightConstants.trap1BlueShotLocation;
//                 if(aprilTagID == 15)
//                     desiredPose = Constants.LimelightConstants.trap2BlueShotLocation;
//                 if(aprilTagID == 16)
//                     desiredPose = Constants.LimelightConstants.trap3BlueShotLocation;
//             }
//         } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
//             if (targetID == TargetID.AMP && aprilTagID == 5)
//                 desiredPose = Constants.LimelightConstants.ampRedShotLocation;
//             else {
//                 if(aprilTagID == 11)
//                     desiredPose = Constants.LimelightConstants.trap1RedShotLocation;
//                 if(aprilTagID == 12)
//                     desiredPose = Constants.LimelightConstants.trap2RedShotLocation;
//                 if(aprilTagID == 13)
//                     desiredPose = Constants.LimelightConstants.trap3RedShotLocation;
//             }
//         }

//         currentPose = Swerve.getInstance().getRobotPose();

//         SmartDashboard.putNumber("AprilTagID", aprilTagID);
//         SmartDashboard.putNumber("DesiredPoseX", desiredPose.getX());
//         SmartDashboard.putNumber("DesiredPoseY", desiredPose.getY());
//         SmartDashboard.putNumber("DesiredPoseTheta", desiredPose.getRotation().getDegrees());
//         SmartDashboard.putNumber("CurrentPoseX", currentPose.getX());
//         SmartDashboard.putNumber("CurrentPoseY", currentPose.getY());
//         SmartDashboard.putNumber("CurrentPoseTheta", currentPose.getRotation().getDegrees());
//     }

//     public ChassisSpeeds getPoseError() {

//         double vxVelocity = 0;
//         double vyVelocity = 0;
//         double omegaVelocity = 0;

//         if(aprilTagID != -1) {

//             vxVelocity = xPID.calculate(currentPose.getX(), desiredPose.getX());
//             vyVelocity = yPID.calculate(currentPose.getY(), desiredPose.getY());
//             omegaVelocity = omegaPID.calculate(currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

//             vxVelocity = MathUtil.clamp(vxVelocity, -3, 3);//m/s
//             vyVelocity = MathUtil.clamp(vyVelocity, -3, 3);//m/s
//             omegaVelocity = MathUtil.clamp(omegaVelocity, -3, 3);//rad/s  
//         }

//         return new ChassisSpeeds(vxVelocity, vyVelocity, omegaVelocity);
//     }


//     /**
//      * Get the lastest pose data field-to-robot from limelight
//      * @return
//      */
//     public PosePacket getPose2dData() {
//         return posePacket;
//     }

//     /**
//      * Transforms RobotPose by SpeakerPose to get relative pose of robot to speaker
//      * @return Center of Robot to Center of Speaker as a Transform3d
//      */
//     public Translation2d getTranslationRobotToSpeaker() {
//         return Swerve.getInstance().getRobotPose().getTranslation()
//             .minus(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()));
//     }

//     /**
//      * Transforms RobotPose by AmpPose to get relative pose of robot to amp
//      * @return Center of Robot to Center of Amp as a Transform3d
//      */
//     public Translation2d getTranslationRobotToAmp() {
//         return Swerve.getInstance().getRobotPose().getTranslation()
//             .minus(AllianceFlipUtil.apply(FieldConstants.ampCenter.toTranslation2d()));
//     }

//     /**
//      * Transforms RobotPose by TrapPose to get relative pose of robot to trap
//      * @return Center of Robot to Center of Trap as a Transform3d
//      */
//     // public Transform3d getPoseRobotToTrap() {
//     //     return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose()),
//     //         AllianceFlipUtil.apply(FieldConstants.Stage.centerPose));
//     // }


//     /**
//      * This returns a translation in the <b>XZ PLANE</b> 
//      * @return Launcher Mouth to Trap Vector
//      */
//     // public Transform3d getTranslationLauncherToTrap() {
//     //     //get the origin to robot center vector
//     //     return new Transform3d(new Pose3d(Swerve.getInstance().getRobotPose())
//     //     //add the robot center to launcher mouth home vector
//     //         .plus(Constants.LauncherConstants.launcherMouthHomeLocationXYZ)
//     //     //add the current launcher mouth vector against the origin launcher mouth vector
//     //         .plus(Wristavator.getInstance().getRobotBaseToLauncherMouthPose()),
//     //     //finally place in origin to trap vector
//     //         AllianceFlipUtil.apply(FieldConstants.Stage.centerPose));
//     // }

//     /**
//      * This returns a translation in the <b>XZ PLANE</b> 
//      * @return Launcher Mouth to Speaker Vector
//      */
//     public Translation2d getTranslationLauncherToSpeaker() {

//         return Wristavator.getInstance().getOriginToLauncherMouthTranslationXZ()
//             .minus(new Translation2d(AllianceFlipUtil.apply(FieldConstants.ampCenter.getX()), FieldConstants.ampCenter.getZ()));
//     }

//     /**
//      * This returns a translation in the <b>YZ PLANE</b> 
//      * @return Launcher Mouth to Amp Vector
//      */
//     public Translation2d getTranslationLauncherToAmp() {
        
//         return Wristavator.getInstance().getOriginToLauncherMouthTranslationYZ()
//             .minus(new Translation2d(FieldConstants.ampCenter.getY(), FieldConstants.ampCenter.getZ()));
//     }

//     public static Limelight getInstance() {
//         if(instance == null)
//             instance = new Limelight();

//         return instance;
//     }
// }
