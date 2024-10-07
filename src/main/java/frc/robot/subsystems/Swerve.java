package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimVisionSystem;
import org.photonvision.simulation.VisionSystemSim;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelight.PosePacket;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase  implements WiredSubsystem {
    
    public static Swerve instance;

    Trigger pathCompleteTriggered;
    BooleanSupplier pathCompleteSupplier;
    boolean isPathComplete = false;

    BooleanSupplier isAimed;
    Trigger swerveAimed;

    Trigger runNow = new Trigger(() -> true);
    PathConstraints constraints;
    Pose3d desiredPathingPose;

    SwerveDrive swerveDrive;

    PIDController ampXDirectionLineupController;

    double prevTimestamp;

    PhotonCamera photonCamera;

    RobotConfig config;

    public enum SwerveState implements InnerWiredSubsystemState {
        HOME,
        X_LOCKED,
        DRIVING,
        AIMING_SPEAKER,
        AIMING_AMP,
        AIMING_TRAP,
        PATHING
    }

    SwerveState swerveState;

    public Swerve() {
        
        pathCompleteSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                if(isPathComplete) {
                    isPathComplete = false;
                    return true;
                } else return false;
            }
        };
        pathCompleteTriggered = new Trigger(pathCompleteSupplier);

        isAimed = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                switch (swerveState) {
                    case AIMING_SPEAKER:
                        return swerveDrive.swerveController.thetaController.atSetpoint();
                
                    case AIMING_AMP:
                        return swerveDrive.swerveController.thetaController.atSetpoint() &&
                            ampXDirectionLineupController.atSetpoint();
                    default:
                        return false;
                }
            }
        };
        swerveAimed = new Trigger(isAimed);

        try {
            swerveDrive = new SwerveParser(Constants.SwerveConstants.swerveDirectory)
                .createSwerveDrive(Constants.SwerveConstants.maxSwerveSpeedMS);
        } catch (IOException e) {
            e.printStackTrace();
        }

        ampXDirectionLineupController = new PIDController(0.02, 0, 0);
        ampXDirectionLineupController.setSetpoint(0);
        
        swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(
            Constants.LimelightConstants.visionMeasurementStdDevs);
            
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
            // AutoBuilder.configureHolonomic(
            // this::getRobotPose, 
            // this::resetRobotPose, 
            // this::getChassisSpeedsRobotRelative, 
            // this::setChassisSpeeds, 
            // Constants.SwerveConstants.swervePathPlannerConfig, 
            // () -> {
            //   // Boolean supplier that controls when the path will be mirrored for the red alliance
            //   // This will flip the path being followed to the red side of the field.
            //   // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            //   var alliance = DriverStation.getAlliance();
            //   if (alliance.isPresent()) {
            //     return alliance.get() == DriverStation.Alliance.Red;
            //   }
            //   return false;
            // }, 
            // this);
                // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getRobotPose, // Robot pose supplier
            this::resetRobotPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeedsRobotRelative, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

        constraints = new PathConstraints(
            5.2, 
            6, 
            Units.degreesToRadians(720),
            Units.degreesToRadians(720));

        swerveState = SwerveState.HOME;

        prevTimestamp = 0;
    }

    public Trigger getPathCompleteTriggered() {
        return pathCompleteTriggered;
    }

    public Trigger getSwerveAimedTrigger() {
        return swerveAimed;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return swerveDrive.getModulePositions();
    }

    public Rotation2d getGyroAngle() {
        return swerveDrive.getOdometryHeading();
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public void resetHeading() {
        double angleOffset = 0;

        if(DriverStation.getAlliance().isPresent()) {
            angleOffset = DriverStation.getAlliance().get() == Alliance.Red ? 180 : 0;
        }
        swerveDrive.resetOdometry(new Pose2d(getRobotPose().getTranslation(), Rotation2d.fromDegrees(angleOffset)));
    }

    /**
     * Field to Robot pose
     * @return
     */
    public Pose2d getRobotPose() {
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        swerveDrive.setChassisSpeeds(speeds);
    }

    public ChassisSpeeds getChassisSpeedsRobotRelative() {
        return swerveDrive.getRobotVelocity();
    }

    public void resetRobotPose(Pose2d resetPose) {
        swerveDrive.resetOdometry(resetPose);
    }

    public void setState(SwerveState swerveState) {
        this.swerveState = swerveState;
    }

    public void xLockSwerve() {
        swerveDrive.lockPose();
    }
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
      swerveDrive.drive(translation,
                        rotation,
                        fieldRelative,
                        false); // Open loop is disabled since it shouldn't be used most of the time
    }

    public void teleopFieldRelativeDrive(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading) {
        //Changed to negative to invert x and y Xbox Controls
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
        if(alliance.get() == DriverStation.Alliance.Red){
        //System.out.println("Red Alliance");
        double xVelocity   = Math.pow(vX.getAsDouble(), 3);
        double yVelocity   = Math.pow(vY.getAsDouble(), 3);
        double angVelocity = Math.pow(heading.getAsDouble()*-1, 3);
        swerveDrive.drive(new Translation2d(xVelocity * Constants.SwerveConstants.maxSwerveSpeedMS, 
            yVelocity * Constants.SwerveConstants.maxSwerveSpeedMS), angVelocity * swerveDrive.getSwerveController().config.maxAngularVelocity,
            true, 
            false);
        } else {
            double xVelocity   = Math.pow(vX.getAsDouble()*-1, 3);
        double yVelocity   = Math.pow(vY.getAsDouble()*-1, 3);
        double angVelocity = Math.pow(heading.getAsDouble()*-1, 3);
        swerveDrive.drive(new Translation2d(xVelocity * Constants.SwerveConstants.maxSwerveSpeedMS, 
            yVelocity * Constants.SwerveConstants.maxSwerveSpeedMS), angVelocity * swerveDrive.getSwerveController().config.maxAngularVelocity,
            true, 
            false);
        }}
        

        
    }

    public void teleopFieldRelativeDriveAiming(DoubleSupplier vX, DoubleSupplier vY, Rotation2d targetToPoseAt) {

        double drivebaseRotationRate = swerveDrive.swerveController.headingCalculate(swerveDrive.getYaw().getRadians(), targetToPoseAt.getDegrees());

        ChassisSpeeds desiredSpeeds = swerveDrive.swerveController.getTargetSpeeds(
            Math.pow(vX.getAsDouble(), 3) * Constants.SwerveConstants.maxSwerveSpeedMS, 
            Math.pow(vY.getAsDouble(), 3) * Constants.SwerveConstants.maxSwerveSpeedMS,
            drivebaseRotationRate, 
            swerveDrive.getYaw().getRadians(), 
            Constants.SwerveConstants.maxSwerveSpeedMS);
        
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        
        translation = SwerveMath.limitVelocity(
            translation, 
            swerveDrive.getFieldVelocity(), 
            swerveDrive.getPose(),
            Constants.SwerveConstants.VelocityControllerLoopTime, 
            Constants.SwerveConstants.RobotMass, 
            List.of(Constants.SwerveConstants.Chassis),
            swerveDrive.swerveDriveConfiguration);

        swerveDrive.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true, false);
    }

    public void teleopFieldRelativePathing(Pose3d desiredPose) {

        // runNow.onTrue(Commands.runOnce(() -> AutoBuilder.pathfindToPose(
        //     desiredPose.toPose2d(), 
        //     constraints, 
        //     0, 
        //     0)).andThen(() -> isPathComplete = true));
    }

    public void setDesiredPathingPose(Pose3d pose) {
        desiredPathingPose = pose;
    }
    

    public void periodic() {

        
        teleopFieldRelativeDrive(()->MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftY(), 
        Constants.SwerveConstants.XboxJoystickDeadband), 
        ()->MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftX(), 
        Constants.SwerveConstants.XboxJoystickDeadband), 
        ()->MathUtil.applyDeadband(RobotContainer.getDriverController().getRightX(), 
        Constants.SwerveConstants.XboxJoystickDeadband));

    // Boolean doRejectUpdate = false;
    // LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LimelightConstants.limelightName);
      
    // if(DriverStation.isDisabled()){

    
    // if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
    // {
    //   if(mt1.rawFiducials[0].ambiguity > .7)
    //   {
    //     doRejectUpdate = true;
    //   }
    //   if(mt1.rawFiducials[0].distToCamera > 3)
    //   {
    //     doRejectUpdate = true;
    //   }
    // }
    // if(mt1.tagCount == 0)
    // {
    //   doRejectUpdate = true;
    // }

    // if(!doRejectUpdate)
    // {
    //   swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
    //   swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    //       mt1.pose,
    //       mt1.timestampSeconds);
    // }
    // }
    
    Boolean doRejectUpdateMt2 = false;
    //Change to Radians?
    LimelightHelpers.SetRobotOrientation(Constants.LimelightConstants.limelightName, swerveDrive.getYaw().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimelightConstants.limelightName);
    //   if(Math.abs(getGyroAngle().getRotations()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    //   {
    //     doRejectUpdate = true;
    //   }
      if(mt2.tagCount == 0)
      {
        doRejectUpdateMt2 = true;
      }
      if(!doRejectUpdateMt2)
      {
        swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        swerveDrive.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
           //swerveDrive.getGyro().getRotation3d().getAngle();
      }
     // '''PHOTON VISION'''
    // AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // photonCamera = new PhotonCamera("PhotonCamera");
    // Transform3d robotToCam = new Transform3d(new Translation3d(0.3492, 0.3746, 0.635), new Rotation3d(0, -12, 0));

    // // Construct PhotonPoseEstimator
    // PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCam);
    
    // var res = photonCamera.getLatestResult();
    //     if (res.hasTargets()) {
    //         var imageCaptureTime = res.getTimestampSeconds();
    //         var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
    //         var camPose = Constants.LimelightConstants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
    //         swerveDrive.addVisionMeasurement(
    //                 camPose.transformBy(robotToCam).toPose2d(), imageCaptureTime);
    //         //swerveDrive.swerveDrivePoseEstimator.update(getGyroAngle(), getSwerveModulePositions());
    //     }
    

        //TODO Uncomment this

        // if(Limelight.getInstance().getPose2dData().timestamp.isPresent() && 
        //     (Limelight.getInstance().getPose2dData().timestamp.get() != prevTimestamp)) {

        //     PosePacket posePacket = Limelight.getInstance().getPose2dData();

        //     if(posePacket.pose2d.isPresent()) {

        //         swerveDrive.addVisionMeasurement(
        //             posePacket.pose2d.get(), 
        //             posePacket.timestamp.get());
                
        //         prevTimestamp = posePacket.timestamp.get();

        //     }  
        // }
    }

    
    

        @Override
    public InnerWiredSubsystemState getState() {
        return swerveState;
    }


    @Override
    public void reportData() {
    }

    public Trigger getPathCompletTriggered() {
        return pathCompleteTriggered;
    }


    public static Swerve getInstance() {
        if (instance == null) 
            instance = new Swerve();
        
        return instance;
    }
}