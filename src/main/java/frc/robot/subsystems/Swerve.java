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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightResults;
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


    Trigger runNow = new Trigger(() -> true);
    PathConstraints constraints;
    Pose3d desiredPathingPose;

   SwerveDrive swerveDrive;

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

        try {
            swerveDrive = new SwerveParser(Constants.SwerveConstants.swerveDirectory)
                .createSwerveDrive(Constants.SwerveConstants.maxSwerveSpeedMS);
        } catch (IOException e) {
            e.printStackTrace();
        }
        //TODO: Uncomment this and fix the errors
        //swerveDrive.stateStdDevs = Constants.LimelightConstants.driveMeasurementStdDevs;
        //swerveDrive.visionMeasurementStdDevs = Constants.LimelightConstants.visionMeasurementStdDevs;
        


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
            
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationOverride);

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

    public SwerveModulePosition[] getSwerveModulePositions() {
        return swerveDrive.getModulePositions();
    }

    public Rotation2d getGyroAngle() {
        return swerveDrive.getYaw();
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    //Pathplanner needs to give us this
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
        //Changed to negative to invert x and y Xbox Controls
        double xVelocity   = Math.pow(vX.getAsDouble()*-1, 3);
        double yVelocity   = Math.pow(vY.getAsDouble()*-1, 3);
        double angVelocity = Math.pow(heading.getAsDouble()*-1, 3);

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

        if(Limelight.getInstance().getPose3dData().timestamp.isPresent() && 
            (Limelight.getInstance().getPose3dData().timestamp.get() != prevTimestamp)) {

            PosePacket posePacket = Limelight.getInstance().getPose3dData();

            if(posePacket.pose3d.isPresent()) {

                swerveDrive.addVisionMeasurement(
                    posePacket.pose3d.get().toPose2d(), 
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

public void autoLineup() {
    
    CommandXboxController driverController = RobotContainer.getDriverController();
    double forward = driverController.getLeftY();
    double strafe = LimelightHelpers.getTX("limelight");
    final Translation2d translation2dAutoLineup = new Translation2d(forward, strafe);
    double rotation = LimelightHelpers.getTX("limelight");
    //double getX = driverController.getRightX();
    if (LimelightHelpers.getLatestResults("limelight") != null) {
        strafe = 0;
        swerveDrive.drive(translation2dAutoLineup, ((DoubleSupplier) Rotation2d.fromDegrees(rotation)).getAsDouble(), true, false);
        //swerveDrive.drive(translation2dAutoLineup, Rotation2d.fromDegrees(rotation).getDegrees(), true, false);
        System.out.println("Auto Lineup Complete");
        //swerveDrive.driveFieldOriented(getChassisSpeedsRobotRelative(),translation2dAutoLineup);
        //Press X
      }
    }

      double limelight_aim_proportional()
      {    
        
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;
    
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
    
        // convert to radians per second for our drive method
        targetingAngularVelocity *= Math.PI;
    
        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;
    
        return targetingAngularVelocity;
      }
    
      // simple proportional ranging control with Limelight's "ty" value
      // this works best if your Limelight's mount height and target mount height are different.
      // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
      double limelight_range_proportional()
      {    
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= Constants.SwerveConstants.maxSwerveSpeedMS;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
      }
    
      public void AutoAlign(boolean fieldRelative) {
        Swerve.getInstance();
        CommandXboxController driverController = RobotContainer.getDriverController();
        final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
        final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
        final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        double xSpeed =
            -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftY(), 0.02))
                * Constants.SwerveConstants.maxSwerveSpeedMS;
    
        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        double ySpeed =
            -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftX(), 0.02))
                * Constants.SwerveConstants.maxSwerveSpeedMS;
    
        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        double rot =
            -m_rotLimiter.calculate(MathUtil.applyDeadband(driverController.getRightX(), 0.02))
                * Math.PI;

        
    
        // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
        if(LimelightHelpers.getLatestResults("limelight") != null)
        {
            //|| LimelightHelpers.getLatestResults("limelight") != null
            //driverController.x().getAsBoolean()
            final var rot_limelight = limelight_aim_proportional();
            rot = rot_limelight;
    
            final var forward_limelight = limelight_range_proportional();
            xSpeed = forward_limelight;
            Translation2d translation2d = new Translation2d(xSpeed, ySpeed);
    
            //while using Limelight, turn off field-relative driving.
            fieldRelative = false;
            swerveDrive.drive(translation2d, rot, fieldRelative, false);
            System.out.println("Successful New Auto Align");
        }
    
       
      }
    
public void turn90Degrees(){
    

    swerveDrive.drive(new Translation2d(0, 0), 90, false, false);
    //0 and 0 for translation2d
}

    public static Swerve getInstance() {
        if (instance == null) 
            instance = new Swerve();
        
        return instance;
    }
}
