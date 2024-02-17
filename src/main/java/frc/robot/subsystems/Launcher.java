package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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

    CANSparkFlex upperLauncherWheels;
    CANSparkFlex lowerLauncherWheels;
    CANSparkMax noteHolder;

    SparkPIDController upperVelocityController;
    SparkPIDController noteHolderPositionController;

    DigitalInput noteInHolder;

    BooleanSupplier noteInSerializerSupplier;
    Trigger noteInSerializerTriggered;

    BooleanSupplier noteShotSupplier;
    Trigger noteShotTriggered;

    BooleanSupplier atRPMSupplier;
    Trigger atRPMTriggered;


    public enum LauncherState implements InnerWiredSubsystemState {
        HOME,
        SERIALIZE_NOTE,
        NOTE_IN_HOLDER,
        AIMING_SPEAKER,
        AIMING_AMP,
        LAUNCHING,
        IDLE_RPM

    }

    LauncherState launcherState;

    double currentRPM;
    double desiredRPM;

    public Launcher() {

        //set up all sensors and motor controllers
        noteInHolder = new DigitalInput(Constants.LauncherConstants.noteinHolderPort);

        upperLauncherWheels = new CANSparkFlex(Constants.LauncherConstants.upperWheelLauncherId, MotorType.kBrushless);
        lowerLauncherWheels = new CANSparkFlex(Constants.LauncherConstants.lowerWheelLauncherId, MotorType.kBrushless);
        noteHolder          = new CANSparkMax(Constants.LauncherConstants.noteHolderId, MotorType.kBrushless);

        upperVelocityController = upperLauncherWheels.getPIDController();
        noteHolderPositionController = noteHolder.getPIDController();

        upperVelocityController.setP(Constants.LauncherConstants.launcherP, 0);
        upperVelocityController.setI(Constants.LauncherConstants.launcherI, 0);
        upperVelocityController.setD(Constants.LauncherConstants.launcherD, 0);
        upperVelocityController.setFF(Constants.LauncherConstants.launcherFF, 0);
        upperVelocityController.setSmartMotionAllowedClosedLoopError(Constants.LauncherConstants.rpmTolerance, 0);

        lowerLauncherWheels.follow(upperLauncherWheels, true);

        noteHolderPositionController.setP(Constants.LauncherConstants.noteP, 0);
        noteHolderPositionController.setI(Constants.LauncherConstants.noteI, 0);
        noteHolderPositionController.setD(Constants.LauncherConstants.noteD, 0);
        noteHolderPositionController.setFF(Constants.LauncherConstants.noteFF, 0);

        //set up state triggers
        noteInSerializerSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return noteInHolder.get();
            }
        };
        noteInSerializerTriggered = new Trigger(noteInSerializerSupplier);

        noteShotSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !noteInHolder.get();
            }
        };
        noteShotTriggered = new Trigger(noteShotSupplier);

        atRPMSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return MathUtil.isNear(desiredRPM, upperLauncherWheels.getEncoder().getVelocity(), Constants.LauncherConstants.rpmTolerance);
            }
        };
        atRPMTriggered = new Trigger(atRPMSupplier);
        
        //intial state
        launcherState = LauncherState.HOME;

        desiredRPM = 0;
        currentRPM = 0;

        //smartdashboard data tab
        launcherTab = Shuffleboard.getTab("launcher");
        launcherTab.addDouble("Launcher RPM", () -> currentRPM);

        upperLauncherWheels.burnFlash();
        lowerLauncherWheels.burnFlash();
        noteHolder.burnFlash();
    }

    public Trigger getNoteSerialized() {
        return noteInSerializerTriggered;
    }

    public Trigger getNoteShot() {
        return noteShotTriggered;
    }

    public Trigger getLauncherAtRPM() {
        return atRPMTriggered;
    }

    public void setRPM(Transform3d transformToTarget) {

        double distanceToTarget = transformToTarget.getTranslation().getNorm();//in meters

        //horner method for fast n degree polynomial calc
        for(int i = Constants.LauncherConstants.launcherPolyCoeffs.size(); i > -1; i--) {
            desiredRPM = Constants.LauncherConstants.launcherPolyCoeffs.get(i) + desiredRPM * distanceToTarget; 
        }

        upperVelocityController.setReference(desiredRPM, ControlType.kSmartVelocity);
    }

    public double getRPM() {
        return upperLauncherWheels.getEncoder().getVelocity();
    }

    public void setState(LauncherState launcherState) {

        this.launcherState = launcherState;
    }

    public void periodic() {
        switch (launcherState) {
            case AIMING_AMP:

                setRPM(Limelight.getInstance().getPoseLauncherToAmp());
                break;
            
            case AIMING_SPEAKER:

                setRPM(Limelight.getInstance().getPoseLauncherToSpeaker());
                break;
            
            case LAUNCHING:
                
                break;
            
            case SERIALIZE_NOTE:

                noteHolderPositionController.setReference(Constants.LauncherConstants.dumbHolderSpeed, ControlType.kSmartVelocity);
                break;

            case NOTE_IN_HOLDER:

                noteHolderPositionController.setReference(0, ControlType.kVelocity);
                break;

            case IDLE_RPM:

                upperVelocityController.setReference(Constants.LauncherConstants.idleRPM, ControlType.kSmartVelocity);
                break;

            case HOME:
            default:
                //scuffed but 0 rpm
                upperVelocityController.setReference(0, ControlType.kSmartVelocity);
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
