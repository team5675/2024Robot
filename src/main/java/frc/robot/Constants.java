package frc.robot;

import java.io.File;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.math.Matter;

public class Constants {
    

    public class SwerveConstants {

        public static final double maxSwerveSpeedMS = Units.feetToMeters(17.6);
        public static final File swerveDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

        public static final double XboxJoystickDeadband = 0.01;

        public static final double VelocityControllerLoopTime = 0.13;//seconds
        public static final double RobotMass = Units.lbsToKilograms(150);
        public static final Matter Chassis = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), RobotMass);

        public static final HolonomicPathFollowerConfig swervePathPlannerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0, 0), 
            new PIDConstants(5, 0, 0.0001), 
            maxSwerveSpeedMS, 
            0.38, 
            new ReplanningConfig(), 
            0.02);

    }

    public class LimelightConstants {

        //Relative to center of robot on the floor
        public static final Pose3d limelightPhysicalLocation = new Pose3d(0.3492, 0.3746, 0.635, new Rotation3d(0, 0, 0));

        public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.02, 0.02, 0.001);
        public static final Matrix<N3, N1> driveMeasurementStdDevs = VecBuilder.fill(0.17, 0.17, 0.001);

    }

    public class LauncherConstants {

        public static final int noteinHolderPort = 3;

        public static final int upperWheelLauncherId = 30;
        public static final int lowerWheelLauncherId = 31;
        public static final int noteHolderId         = 32;

        public static final double launcherP = 0.000023;
        public static final double launcherI = 0.0;
        public static final double launcherD = 0.0;
        public static final double launcherFF = 0.0014;//in volts

        public static final double noteP = 0.4;
        public static final double noteI = 0.0;
        public static final double noteD = 0.0;
        public static final double noteFF = 0.2;//in volts

        public static final double idleRPM = 0;//RPM

        //Will come from tuing RPM range
        public static List<Double> launcherSpeakerPolyCoeffs = Arrays.asList(1.222, -0.234, 0.235346);
        public static List<Double> launcherAmpPolyCoeffs = Arrays.asList(1.222, -0.234, 0.235346);

        public static final double rpmTolerance = 50; //rpm

        public static final double dumbHolderSpeed = 200; //rpm
        public static final double launchingHolderSpeed = 2000; //rpm

        //Relative to center of robot on the floor
        public static final Translation3d launcherMouthHomeLocationXYZ = 
            new Translation3d(0,0,0.343);
       
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

    public class WristavatorConstants {

        public static final int wristID = 40;
        public static final int elevatorID = 5;

        public static final int wristLimitSwitchId = 4;
        public static final int elevatorLimitSwitchID = 5;

        public static final double elevatorTolerance = 0.005;//meters
        public static final double wristTolerance = 0.01;//radians

        public static final double elevatorP = 0.02;
        public static final double elevatorI = 0;
        public static final double elevatorD = 0;

        public static final double wristP = 0.0002;
        public static final double wristI = 0;
        public static final double wristD = 0;

        public static final double wristKS = 0.22;
        public static final double wristKG = 0.25;
        public static final double wristKV = 3.66;

        public static final double elevatorKS = 0.22;
        public static final double elevatorKG = 0.02;
        public static final double elevatorKV = 153.45; //assuming 15lb load and 100:1 reduction

        //Meters, 0 is home, rotation is 0 from horizontal
        //public static final Translation2d wristavatorIntakePose = new Translation2d(0, Rotation2d.fromDegrees(0));
        //public static final Translation2d wristavatorAmpPose = new Translation2d(0.152, Rotation2d.fromDegrees(95));
        //public static final Translation2d wristavatorTrapPose = new Translation2d(1, Rotation2d.fromDegrees(45));

        public static final Translation2d wristavatorHomePose = new Translation2d(0, Rotation2d.fromDegrees(55));
        public static final Translation2d wristavatorSpeakerProtectedPose = new Translation2d(0, Rotation2d.fromDegrees(35));

        public static final Rotation2d wristZeroOffset = Rotation2d.fromDegrees(55);//degrees, what the wrist is actually at when limit switch hit
        public static final double elevatorZeroOffset = 0.02;//meters, what elevator is actaully at when limit switch hits

        public static final int wristPositionOffset = 360; //Should this be zero?
        public static final int elevatorPositionOffset = 1;

        //m/s and m/s^s
        public static final TrapezoidProfile.Constraints wristProfileConstraints = new TrapezoidProfile.Constraints(1, 0.3);
        public static final TrapezoidProfile.Constraints elevatorProfileConstraints = new TrapezoidProfile.Constraints(0.5, 0.2);
    }

    public class IntakeConstants {

        public static final int intakeMotorID = 25;
        public static final int IntakeProxPort = 2;
        public static final int IntakeSpeedRPM = 1238;
        public static final int OuttakeSpeedRPM = -1238;
        
    }

    public class ClimberConstants {

        public static final int climberMotorID = 27;
        public static final int servoID = 2;

        public static final int latchPulseTimeClosed = 1500;
        public static final int latchPulseTimeOpen = 1200;

        public static final double climbP = 0.02;
        public static final double climbI = 0.0;
        public static final double climbD = 2e-6;

        public static final Rotation2d climbExtended = Rotation2d.fromDegrees(45); //degrees
        public static final Rotation2d climbRetracted = Rotation2d.fromDegrees(0); //degrees
    }

    public class BlowerConstants {
            
            public static final int blowerMotorID = 26;
            public static final double BlowerSpeedPercentage = 0.5;

    }

    public class LEDConstants{
        public static final int blinkinID = 1;
        public static final double LEDRainbow = -0.99;
        public static final double LEDOceanRainbow = -0.95;
        public static final double LEDOrange = 0.65;
        public static final double LEDBlue = 0.87;
        public static final double LEDRed = 0.59;
        //Change the default pattern by holding the mode button, then holding the other button
    }
    /**
     * All constants assume Blue side origin, and are in meters
     */
    public class FieldConstantsDep {

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
