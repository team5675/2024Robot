package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveDrive;

public class ConnorLineup extends Command {

    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry horizontalOffset = limelightTable.getEntry("tx");
    private NetworkTableEntry verticalOffset = limelightTable.getEntry("ty");
    private Swerve drive = Swerve.getInstance();
    private double rotateClockwise = 0.3;
    private double rotateNotClockwise = -0.3;
    private double kLimeLightAngle = 45.0;
    private double kAprilTagHeight = 1.2192;
    private double kLimelightHeight = 0.6096;
    private Translation2d MoveGood = new Translation2d(0, 0);

    private double kHeightDiff = (kAprilTagHeight - kLimelightHeight);
    private double kTargetDistance = 1.2192;
    private double kDistance;
    private double kLimeLightVerticalAngle;
    private double kLimeLightHorizontalAngle;

    private final PIDController xMovePID = new PIDController(5, 0, 0);
    private final PIDController yMovePID = new PIDController(5, 0, 0);

    public ConnorLineup() {
        xMovePID.setSetpoint(0.0);
        yMovePID.setSetpoint(0.0);
    }

    private double calculateXMove(double horizontalAngle) {
        return -xMovePID.calculate(horizontalAngle, 0.0);
    }

    private double calculateYMove(double verticalAngle, double kTargetDistance) {
    kDistance = kHeightDiff / Math.tan(Math.toRadians(kLimeLightAngle) + Math.toRadians(kLimeLightVerticalAngle));
    double error = kTargetDistance - kDistance;
    return yMovePID.calculate(error, 0.0);
}

    @Override
    public void execute() {
        if (limelightTable.getEntry("tv").getDouble(0) != 0) {

            kLimeLightVerticalAngle = verticalOffset.getDouble(0);
            kLimeLightHorizontalAngle = horizontalOffset.getDouble(0);

            // Calculate the required distance to move in x and y axis
            xMove = calculateXMove(kLimeLightHorizontalAngle);
            yMove = calculateYMove(kLimeLightVerticalAngle, kTargetDistance);

            MoveGood = new Translation2d(xMove, yMove);

            // Drive the robot with the calculated xMove and yMove
            drive.drive(MoveGood, 0, false);

            if (kTargetDistance - kDistance =< 0.01 && kLimeLightHorizontalAngle =< 0.1 ){
                System.out.println("Lineup Complete");
            } else { System.out.println("Lineup Failed");
            }

        } else {
            System.out.println("No April Tag Detected");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Command ends, stop the movement
        drive.drive(new Translation2d(0, 0), 0, false);
    }
}