package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase  implements WiredSubsystem {
    
    public static Swerve instance;

    Trigger pathCompleteTriggered;
    BooleanSupplier pathCompleteSupplier;
    boolean isPathComplete = false;

    Trigger runNow = new Trigger(() -> true);
    PathConstraints constraints;
    Pose3d desiredPathingPose;

    SwerveDrive swerveDrive;

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

        try {
            swerveDrive = new SwerveParser(Constants.SwerveConstants.swerveDirectory)
                .createSwerveDrive(Constants.SwerveConstants.maxSwerveSpeedMS);
        } catch (IOException e) {
            e.printStackTrace();
        }

        swerveDrive.stateStdDevs = Constants.LimelightConstants.driveMeasurementStdDevs;
        swerveDrive.visionMeasurementStdDevs = Constants.LimelightConstants.visionMeasurementStdDevs;


        AutoBuilder.configureHolonomic(
            this::getRobotPose, 
            this::resetRobotPose, 
            this::getChassisSpeedsRobotRelative, 
            this::setChassisSpeeds, 
            Constants.SwerveConstants.swervePathPlannerConfig, 
            () -> false, 
            this);

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationOverride);

        constraints = new PathConstraints(
            5.2, 
            6, 
            Units.degreesToRadians(720),
            Units.degreesToRadians(720));

        swerveState = SwerveState.HOME;
    }

    public Trigger getPathCompleteTriggered() {
        return pathCompleteTriggered;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return swerveDrive.getModulePositions();
    }

    public Rotation2d getGyroAngle() {
        return swerveDrive.getYaw();
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    //Pathplanner needs to givve us this
    public Pose2d getInitialRobotPose() {
        return new Pose2d();
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

    public Optional<Rotation2d> getRotationOverride() {
        
        if(swerveState == SwerveState.AIMING_SPEAKER) {
            return Optional.of(Limelight.getInstance().getPoseRobotToSpeaker().getRotation().toRotation2d());
        }

        if(swerveState == SwerveState.AIMING_AMP) {
            return Optional.of(Limelight.getInstance().getPoseRobotToAmp().getRotation().toRotation2d());
        }

        if(swerveState == SwerveState.AIMING_TRAP) {
            return Optional.of(Limelight.getInstance().getPoseRobotToTrap().getRotation().toRotation2d());
        }
        
        return Optional.empty();
    }

    public void xLockSwerve() {
        swerveDrive.lockPose();
    }

    public void teleopFieldRelativeDrive(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading) {

        double xVelocity   = Math.pow(vX.getAsDouble(), 3);
        double yVelocity   = Math.pow(vY.getAsDouble(), 3);
        double angVelocity = Math.pow(heading.getAsDouble(), 3);

        swerveDrive.drive(new Translation2d(xVelocity * Constants.SwerveConstants.maxSwerveSpeedMS, 
            yVelocity * Constants.SwerveConstants.maxSwerveSpeedMS), angVelocity * swerveDrive.getSwerveController().config.maxAngularVelocity,
            true, 
            false);
    }

    public void teleopFieldRelativeDriveAiming(DoubleSupplier vX, DoubleSupplier vY, Transform3d targetToPoseAt) {

        double drivebaseRotationRate = swerveDrive.swerveController.headingCalculate(swerveDrive.getYaw().getRadians(), targetToPoseAt.getRotation().getZ());

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

                teleopFieldRelativeDriveAiming(
                    () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftY(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                    () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftX(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                    Limelight.getInstance().getPoseRobotToSpeaker());

                break;

            case AIMING_AMP:

                teleopFieldRelativeDriveAiming(
                    () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftY(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                    () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftX(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                    Limelight.getInstance().getPoseRobotToAmp());

                break;

            case AIMING_TRAP:

                teleopFieldRelativeDriveAiming(
                    () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftY(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                    () -> MathUtil.applyDeadband(RobotContainer.getDriverController().getLeftX(),  Constants.SwerveConstants.XboxJoystickDeadband), 
                    Limelight.getInstance().getPoseRobotToTrap());

                break;

            case PATHING:

                teleopFieldRelativePathing(null);

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

        if(Limelight.getInstance().getPose3dData().isPresent()) {

            swerveDrive.addVisionMeasurement(
                Limelight.getInstance().getPose3dData().get().toPose2d(), 
                Limelight.getInstance().getTimestampData());
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
