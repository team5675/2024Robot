package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Launcher implements WiredSubsystem {
    
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
    Rotation2d launcherAngle;

    boolean isAtDesiredRPM;
    boolean isAtDesiredAngle;
    boolean isHome;


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
                return isAtDesiredRPM && isAtDesiredAngle;
            }
        };
        aimingCompleteTrigger = new Trigger(aimingCompleteSupplier);

        launcherShotSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                //TODO: Update launcher shot note condition
                return isAtDesiredRPM && isAtDesiredAngle;
            }
        };
        launcherShotTrigger = new Trigger(launcherShotSupplier);

        //intial state
        launcherState = LauncherState.HOME;

        isAtDesiredAngle = false;
        isAtDesiredRPM = false;
        isHome = false;

        launcherRPM = 0;
        launcherAngle = Constants.LauncherConstants.initialLauncherAngle;

        //smartdashboard data tab
        launcherTab = Shuffleboard.getTab("launcher");
        launcherTab.addDouble("Launcher RPM", () -> launcherRPM);
    }

    public Trigger getLauncherProxTriggered() {
        return launcherProxTriggered;
    }

    public Trigger getAimingCompleteTriggered() {
        return aimingCompleteTrigger;
    }

    public Trigger getLauncherShotTriggered() {
        return launcherShotTrigger;
    }

    public void setState(LauncherState launcherState) {

        this.launcherState = launcherState;

        //TODO: Finish state transitions and flesh out features
        switch (launcherState) {
            case HOME:
                
                break;
        
            default:
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
