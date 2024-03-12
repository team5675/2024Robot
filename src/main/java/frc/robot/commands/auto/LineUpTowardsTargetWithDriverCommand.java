package frc.robot.commands.auto;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveDrive;


public class LineUpTowardsTargetWithDriverCommand extends Command {

    SwerveDrive drive;
    AprilTag aprilTag;
    
    private final SlewRateLimiter fwdLimiter = new SlewRateLimiter(8);
    CommandXboxController xboxControllerDriver = RobotContainer.driverController;
    
    public LineUpTowardsTargetWithDriverCommand(SwerveDrive drive, AprilTag aprilTag, CommandXboxController xboxControllerDriver) {

        addRequirements(Swerve.getInstance());

        this.drive = drive;
        this.aprilTag = aprilTag;
        this.xboxControllerDriver = xboxControllerDriver;
    }
    @Override
    public void initialize() {
    }
    @Override
    public void execute() {

        //TODO: TEST YAW LINEUP PLEASE
        double desiredStrafe = 0;
        double desiredYaw = 0;
        //double desiredYaw = (90 - drive.getYaw()) * 0.005;

        if(LimelightHelpers.getLatestResults("limelight") != null) { 

            desiredStrafe = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

            desiredYaw = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
            
        }

        drive.driveFieldOriented(fwdLimiter.calculate(
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
