package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Constants {
    

    public class SwerveConstants {

    }

    public class LimelightConstants {

        public static final Pose3d limelightPhysicalLocation = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

        public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.02, 0.02, 0.001);
        public static final Matrix<N3, N1> driveMeasurementStdDevs = VecBuilder.fill(0.17, 0.17, 0.001);

    }

    public class LauncherConstants {

        public static final int launcherProxPort = 4;

        public static final double launcherAmpRPM = 3000;

        public static final Rotation2d initialLauncherAngle = Rotation2d.fromDegrees(30);

        public static final Pose3d launcherMouthLocationXYZ = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

        public static final Pose3d speakerLocationXYZ = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        /*
         *  --------
         *  \      |
         * **\__   |**** Location is the front right corner towards side of field and stage
         *      |  |  
         *      |  |
         *      |__|
         */

    }

    public class IntakeConstants {

        public static final int IntakeProxPort = 2;
        public static final int IntakeSpeedRPM = 2500;
        public static final int OuttakeSpeedRPM = -2500;
        
    }
}
