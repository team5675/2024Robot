package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RobotPoseEstimator extends SubsystemBase {
    
    public static RobotPoseEstimator instance;

    SwerveDrivePoseEstimator swerveEstimator;

    Optional<Pose2d> estimatedRobotPose2d;

    public RobotPoseEstimator() {

        swerveEstimator = new SwerveDrivePoseEstimator(
            Swerve.getInstance().getKinematics(), 
            Swerve.getInstance().getGyroAngle(), 
            Swerve.getInstance().getSwerveModulePositions(), 
            Swerve.getInstance().getInitialRobotPose(),
            Constants.LimelightConstants.driveMeasurementStdDevs,
            Constants.LimelightConstants.visionMeasurementStdDevs);

        swerveEstimator.setVisionMeasurementStdDevs(null);

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
                Swerve.getInstance().getSwerveModulePositions()));
    }

    /**
     * Resets robot pose to desired pose, angle, and module positions
     * @param angle the angle of the robot
     * @param modulePositions the positions of the modules
     * @param pose the pose of the robot
     */
    public void resetRobotPose(Rotation2d angle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        swerveEstimator.resetPosition(angle, modulePositions, pose);
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
