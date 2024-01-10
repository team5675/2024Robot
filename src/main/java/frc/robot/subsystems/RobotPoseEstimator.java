package frc.robot.subsystems;

import java.util.Optional;

import javax.swing.text.html.Option;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;

public class RobotPoseEstimator {
    
    public static RobotPoseEstimator instance;

    SwerveDrivePoseEstimator swerveEstimator;

    Optional<Pose2d> estimatedRobotPose2d;

    public RobotPoseEstimator() {

        //TODO: Add swerveKinematics, gyro angles, module postions, and initial pose (from auto)
        swerveEstimator = new SwerveDrivePoseEstimator(null, null, null, null);
        
        estimatedRobotPose2d = Optional.empty();
    }
    
    public void periodic() {

        if(Limelight.getInstance().getPose3dData().isPresent()) {

            swerveEstimator.addVisionMeasurement(
                Limelight.getInstance().getPose3dData().get().toPose2d(), 
                Limelight.getInstance().getTimestampData());
        }

        estimatedRobotPose2d = Optional.of(swerveEstimator.updateWithTime(
                Timer.getFPGATimestamp(), 
                Rotation2d.fromDegrees(0), 
                null));
    }

    public Optional<Pose2d> getRobotPose() {
        return estimatedRobotPose2d;
    }

    public static RobotPoseEstimator getInstance() {
        if(instance == null)
            instance = new RobotPoseEstimator();
        
        return instance;
    }
}
