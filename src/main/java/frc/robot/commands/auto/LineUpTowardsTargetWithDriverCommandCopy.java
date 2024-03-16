package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveDrive;


public class LineUpTowardsTargetWithDriverCommandCopy extends Command {

    static NetworkTable limelightTable;
    static NetworkTableEntry horizontalOffset;
    static NetworkTableEntry verticalOffset;
    static NetworkTableEntry verticalAngleOffset;
    static NetworkTableEntry isTarget;
    Swerve drive;
    AprilTagDetection aprilTag;
    
    private final SlewRateLimiter fwdLimiter = new SlewRateLimiter(8);
    CommandXboxController xboxControllerDriver = RobotContainer.driverController;
    
    public LineUpTowardsTargetWithDriverCommandCopy() {

        addRequirements(Swerve.getInstance());

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        horizontalOffset = limelightTable.getEntry("tx");
        verticalOffset = limelightTable.getEntry("ty");
        isTarget = limelightTable.getEntry("tv");
        
        this.drive = drive;
    this.aprilTag = aprilTag;
    }
    @Override
    public void initialize() {
    }
    @Override
    public void execute() {
        double desiredStrafe = 0;
        double forwardDistance = ((Constants.VISION_TARGET_HEIGHT - 0.635) / Math.tan(Math.toRadians(-12 + verticalOffset.getDouble(0)))) / 12;
        
      
        if(isTarget.getDouble(0) != 0) { 

            desiredStrafe = horizontalOffset.getDouble(0) * 0.025;
            
        }
        //drive.teleopFieldRelativeDrive(desiredStrafe, forwardDistance, 0);
        drive.drives(fwdLimiter.calculate(
            MathUtil.applyDeadband(
                -xboxControllerDriver.getRawAxis(1), 
                0.075)), 
                
                desiredStrafe, 
                
            MathUtil.applyDeadband(
                -xboxControllerDriver.getRawAxis(4), 
                0.075));
    }
    @Override
    public void end(boolean interrupted) {
    }
}
