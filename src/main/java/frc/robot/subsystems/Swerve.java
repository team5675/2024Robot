package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Swerve extends SubsystemBase  implements WiredSubsystem {
    
    public static Swerve instance;

    Trigger pathCompleteTriggered;
    BooleanSupplier pathCompleteSupplier;

    SwerveModulePosition[] wheelPositions;
    SwerveDriveKinematics kinematics;

    public enum SwerveState implements InnerWiredSubsystemState {
        HOME,
        X_LOCKED,
        DRIVING,
        PATHING
    }

    SwerveState swerveState;

    public Swerve() {

        pathCompleteSupplier = new BooleanSupplier() {
            //TODO: Add stop condition from Pathplanner
            @Override
            public boolean getAsBoolean() {
                return true;
            }
        };
        pathCompleteTriggered = new Trigger(pathCompleteSupplier);
    }

    public Trigger getPathCompleteTriggered() {
        return pathCompleteTriggered;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return wheelPositions;
    }

    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(0);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    //Pathplanner needs to givve us this
    public Pose2d getInitialRobotPose() {
        return new Pose2d();
    }

    public void setState(SwerveState swerveState) {
        this.swerveState = swerveState;
    }

    public void periodic() {
        switch (swerveState) {

            case X_LOCKED:

                break;
            case DRIVING:

                break;
            case PATHING:

                break;
            case HOME:
            default:
            
                break;
        }
    }

        @Override
    public InnerWiredSubsystemState getState() {
        return swerveState;
    }


    @Override
    public void reportData() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reportData'");
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
