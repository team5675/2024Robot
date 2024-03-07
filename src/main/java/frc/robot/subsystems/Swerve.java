package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.RobotContainer;
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


        AutoBuilder.configureHolonomic(
            this::getRobotPose, 
            this::resetRobotPose, 
            this::getChassisSpeedsRobotRelative, 
            this::setChassisSpeeds, 
            Constants.SwerveConstants.swervePathPlannerConfig, 
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
            this);

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

    public void teleopFieldRelativeDrive(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading) {
        //Changed to negative to invert x and y Xbox Controls
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
        if(alliance.get() == DriverStation.Alliance.Red){
        System.out.println("Red Alliance");
        double xVelocity   = Math.pow(vX.getAsDouble(), 3);
        double yVelocity   = Math.pow(vY.getAsDouble(), 3);
        double angVelocity = Math.pow(heading.getAsDouble(), 3);
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

        runNow.onTrue(Commands.runOnce(() -> AutoBuilder.pathfindToPose(
            desiredPose.toPose2d(), 
            constraints, 
            0, 
            0)).andThen(() -> isPathComplete = true));
    }

    public void setDesiredPathingPose(Pose3d pose) {
        desiredPathingPose = pose;
    }
    

    public void periodic() {
        switch (swerveState) {

            case X_LOCKED:

                xLockSwerve();
                break;
        
            case AIMING_SPEAKER:

                // teleopFieldRelativeDriveAiming(
                //     () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftY(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                //     () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftX(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                //     Limelight.getInstance().getTranslationRobotToSpeaker().getAngle());

                break;

            case AIMING_AMP:

                // teleopFieldRelativeDriveAiming(
                //     () -> ampXDirectionLineupController.calculate(Limelight.getInstance().getTranslationRobotToAmp().getX()), 
                //     () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftX(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                //     Limelight.getInstance().getTranslationRobotToAmp().getAngle());

                break;

            case AIMING_TRAP:

                // teleopFieldRelativeDriveAiming(
                //     () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftY(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                //     () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftX(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                //     Limelight.getInstance().getTranslationLauncherToSpeaker().getAngle());

                break;

            case PATHING:

                // teleopFieldRelativePathing(null);

                break;

            case HOME:
            case DRIVING:
            default:

                teleopFieldRelativeDrive(
                    () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftY(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                    () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftX(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                    () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getRightX(), Constants.SwerveConstants.XboxJoystickDeadband));

                break;
        }

        if(Limelight.getInstance().getPose3dData().timestamp.isPresent() && 
            (Limelight.getInstance().getPose3dData().timestamp.get() != prevTimestamp)) {

            PosePacket posePacket = Limelight.getInstance().getPose3dData();

            if(posePacket.pose3d.isPresent()) {

                swerveDrive.addVisionMeasurement(
                    posePacket.pose2d.get(), 
                    posePacket.timestamp.get());
                
                prevTimestamp = posePacket.timestamp.get();

            }  
        }
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
