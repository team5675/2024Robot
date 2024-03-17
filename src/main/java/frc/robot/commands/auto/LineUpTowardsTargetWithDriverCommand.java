package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveDrive;


public class LineUpTowardsTargetWithDriverCommand extends Command {

    private final Swerve drive;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;
  private double heading;
  private double lastGoodHeading;
    
    private final SlewRateLimiter fwdLimiter = new SlewRateLimiter(8);
    CommandXboxController xboxControllerDriver = RobotContainer.driverController;
    
    public LineUpTowardsTargetWithDriverCommand(Swerve swerve, DoubleSupplier translationX, DoubleSupplier translationY) {

        addRequirements(Swerve.getInstance());

        this.drive = swerve;
    this.translationX = translationX;
    this.translationY = translationY;
    lastGoodHeading = 0;
    }
    @Override
    public void initialize() {
    }
    @Override
    public void execute() {
       // if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean() == NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").setInteger(7))
        if(LimelightHelpers.getLatestResults("limelight") != null) { 
            while(Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)) > 0){

         //desiredStrafe = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
            System.out.println("Line up complete");
            heading = -NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)/90;

            drive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * Constants.SwerveConstants.maxSwerveSpeedMS,
    Math.pow(translationY.getAsDouble(), 3) * Constants.SwerveConstants.maxSwerveSpeedMS),
    heading * Constants.SwerveConstants.maxSwerveSpeedMS,
false);
            }
        }else{
            heading = 0;
            System.out.println("Failed to see target");
        }
        lastGoodHeading = heading;

          
      

        // drive.drive(fwdLimiter.calculate(
        //     MathUtil.applyDeadband(
        //         -xboxControllerDriver.getRawAxis(1), 
        //         0.075)), 
                
        //         desiredStrafe, 
                
        //     MathUtil.applyDeadband(
        //         -xboxControllerDriver.getRawAxis(4), 
        //         0.075));
    }
    @Override
    public boolean isFinished() {
        
        //return when note in launcher
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) == 0;
        //false
    }
    @Override
    public void end(boolean interrupted) {
    }
}
