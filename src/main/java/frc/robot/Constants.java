package frc.robot;

import java.io.File;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import swervelib.math.Matter;

public class Constants {
    

    public class SwerveConstants {

        public static final double maxSwerveSpeedMS = Units.feetToMeters(17);
        public static final File swerveDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

        public static final double XboxJoystickDeadband = 0.01;

        public static final double VelocityControllerLoopTime = 0.13;//seconds
        public static final double RobotMass = Units.lbsToKilograms(150);
        public static final Matter Chassis = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), RobotMass);

        public static final HolonomicPathFollowerConfig swervePathPlannerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0, 0), 
            new PIDConstants(5, 0, 0.01), 
            maxSwerveSpeedMS, 
            0.38, 
            new ReplanningConfig(), 
            0.02);

    }

    public class LimelightConstants {

        //Relative to center of robot on the floor
        public static final Pose3d limelightPhysicalLocation = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

        public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.02, 0.02, 0.001);
        public static final Matrix<N3, N1> driveMeasurementStdDevs = VecBuilder.fill(0.17, 0.17, 0.001);

    }

    public class LauncherConstants {

        public static final int launcherProxPort = 4;

        public static final double launcherAmpRPM = 3000;

        public static final Rotation2d initialLauncherAngle = Rotation2d.fromDegrees(30);

        public static final double rpmTolerance = 0.5; //rpm
        public static final double angleTolerance = 0.01; //radians

        //Relative to center of robot on the floor
        public static final Transform3d launcherMouthLocationXYZ = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
       
        /* Robot Relative Plane
         * BL(-,+)        FL(+,+)
         * ----------------
         * |              |    Y
         * |              |    ^
         * |              |    |
         * |              |   Z.->X
         * |              |
         * ----------------
         * BR(-,-)        FR(+,-)
         */



    }

    public class IntakeConstants {

        public static final int IntakeProxPort = 2;
        public static final int IntakeSpeedRPM = 2500;
        public static final int OuttakeSpeedRPM = -2500;
        
    }

    /**
     * All constants assume Blue side origin, and are in meters
     */
    public class FieldConstants {

        public static Pose3d getAllianceBasedSpeakerPose() {
            return DriverStation.getAlliance().isPresent() ? 
                DriverStation.getAlliance().get() == Alliance.Red ? 
                SpeakerOpeningCenterPoseRed : 
                SpeakerOpeningCenterPoseBlue : 
                new Pose3d();
        }

        public static Pose3d getAllianceBasedAmpPose() {
            return DriverStation.getAlliance().isPresent() ? 
                DriverStation.getAlliance().get() == Alliance.Red ? 
                AmpOpeningCenterPoseRed : 
                AmpOpeningCenterPoseBlue : 
                new Pose3d();
        }

        public static Pose3d getAllianceBasedTrapPose() {
            return DriverStation.getAlliance().isPresent() ? 
                DriverStation.getAlliance().get() == Alliance.Red ? 
                TrapOpeningCenterPoseRed : 
                TrapOpeningCenterPoseBlue : 
                new Pose3d();
        }

        public static final Pose3d SpeakerOpeningCenterPoseBlue = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        public static final Pose3d AmpOpeningCenterPoseBlue     = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        public static final Pose3d TrapOpeningCenterPoseBlue    = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

        public static final Pose3d SpeakerOpeningCenterPoseRed = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        public static final Pose3d AmpOpeningCenterPoseRed     = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        public static final Pose3d TrapOpeningCenterPoseRed    = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

        /* Field Relative Plane
         *         2                                        5
         *  --------------------------------------------------------
         *  |                      Y                               |
         *  |                      ^                               |
         *1 |                      |                               | 4
         *  |              3      Z.->X               6            |
         *  |                                                      |
         *  |                                                      |
         *  |                                                      |
         *  ---------------------------------------------------------
         * 1-Blue side Speaker
         * 2-Blue side Amp
         * 3-Blue side Trap
         */
    }
}
