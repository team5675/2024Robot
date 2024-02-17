package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Launcher extends SubsystemBase implements WiredSubsystem {
    
    public static Launcher instance;

    ShuffleboardTab launcherTab;

    DigitalInput launcherProx;

    BooleanSupplier launcherProxSupplier;
    Trigger launcherProxTriggered;

    BooleanSupplier aimingCompleteSupplier;
    Trigger aimingCompleteTrigger;

    BooleanSupplier launcherShotSupplier;
    Trigger launcherShotTrigger;

    public enum LauncherState implements InnerWiredSubsystemState {
        HOME,
        AIMING_SPEAKER_LAZY,
        AIMING_SPEAKER_REAL,
        AIMING_AMP,
        LAUNCHING,

    }

    LauncherState launcherState;

    double launcherRPM;

    boolean isAtDesiredRPM;


    public Launcher() {

        //set up all sensors and motor controllers
        launcherProx = new DigitalInput(Constants.LauncherConstants.launcherProxPort);

        //set up state triggers
        launcherProxSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return launcherProx.get();
            }
        };
        launcherProxTriggered = new Trigger(launcherProxSupplier);


        aimingCompleteSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isAtDesiredRPM;
            }
        };
        aimingCompleteTrigger = new Trigger(aimingCompleteSupplier);

        launcherShotSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                //TODO: Update launcher shot note condition
                return isAtDesiredRPM;
            }
        };
        launcherShotTrigger = new Trigger(launcherShotSupplier);

        //intial state
        launcherState = LauncherState.HOME;

        isAtDesiredRPM = false;

        launcherRPM = 0;

        //smartdashboard data tab
        launcherTab = Shuffleboard.getTab("launcher");
        launcherTab.addDouble("Launcher RPM", () -> launcherRPM);
    }

    public Trigger getLauncherProxTriggered() {
        return launcherProxTriggered;
    }

    public Trigger getLauncherShotTriggered() {
        return launcherShotTrigger;
    }

    public void setHomeSpeed() {
        //set to home speed
    }

    public void setRPM(Transform3d transformToTarget) {

        launcherRPM = transformToTarget.getTranslation().getNorm();//in meters

        //put through magic equation to poop out rpm needed
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(0);
    }

    public double getRPM() {
        return 0;
    }

    public void setState(LauncherState launcherState) {

        this.launcherState = launcherState;
    }

    public void periodic() {
        switch (launcherState) {
            case AIMING_AMP:
                setRPM(Limelight.getInstance().getPoseLauncherToAmp());

                isAtDesiredRPM = MathUtil.isNear(
                    launcherRPM, 
                    getRPM(), 
                    Constants.LauncherConstants.rpmTolerance);

                break;

            case AIMING_SPEAKER_LAZY:

                setRPM(Limelight.getInstance().getPoseLauncherToSpeaker());
                isAtDesiredRPM = false;

                break;
            
            case AIMING_SPEAKER_REAL:

                setRPM(Limelight.getInstance().getPoseLauncherToSpeaker());

                isAtDesiredRPM = MathUtil.isNear(
                    launcherRPM, 
                    getRPM(), 
                    Constants.LauncherConstants.rpmTolerance);

                break;
            
            case LAUNCHING:
                
                break;
            case HOME:
            default:

                setHomeSpeed();

                isAtDesiredRPM = false;
                break;
        }
    }

    @Override
    public InnerWiredSubsystemState getState() {
        return launcherState;
    }

    @Override
    public void reportData() {
        
        SmartDashboard.putString("Launcher State", launcherState.toString());
    }

    public static Launcher getInstance() {
        if (instance == null) 
            instance = new Launcher();

        return instance;
    }
}
