package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
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

    public CANSparkFlex upperLauncherWheels;
    public CANSparkFlex lowerLauncherWheels;
    public CANSparkMax noteHolder;

    public SparkPIDController upperVelocityController;
    public SparkPIDController lowerVelocityController;
    SparkPIDController noteHolderPositionController;

    public DigitalInput noteInHolder;

    BooleanSupplier noteInSerializerSupplier;
    Trigger noteInSerializerTriggered;

    BooleanSupplier noteShotSupplier;
    Trigger noteShotTriggered;

    BooleanSupplier atRPMSupplier;
    Trigger atRPMTriggered;

    BooleanSupplier proximitySensor;
    


    public enum LauncherState implements InnerWiredSubsystemState {
        HOME,
        SERIALIZE_NOTE,
        NOTE_IN_HOLDER,
        AIMING_SPEAKER,
        AIMING_AMP,
        LAUNCHING,
        IDLE_RPM, GATE_WHEEL

    }

    LauncherState launcherState;

    double currentRPM;
    public double desiredRPM;

    public Launcher() {

        //set up all sensors and motor controllers
        noteInHolder = new DigitalInput(Constants.LauncherConstants.noteinHolderPort);

        upperLauncherWheels = new CANSparkFlex(Constants.LauncherConstants.upperWheelLauncherId, MotorType.kBrushless);
        lowerLauncherWheels = new CANSparkFlex(Constants.LauncherConstants.lowerWheelLauncherId, MotorType.kBrushless);
        noteHolder          = new CANSparkMax(Constants.LauncherConstants.noteHolderId, MotorType.kBrushless);

        upperVelocityController = upperLauncherWheels.getPIDController();
        lowerVelocityController = lowerLauncherWheels.getPIDController();
        noteHolderPositionController = noteHolder.getPIDController();

        upperVelocityController.setP(Constants.LauncherConstants.launcherP, 0);
        upperVelocityController.setI(Constants.LauncherConstants.launcherI, 0);
        upperVelocityController.setD(Constants.LauncherConstants.launcherD, 0);
        upperVelocityController.setFF(Constants.LauncherConstants.launcherFF, 0);
        //upperVelocityController.setSmartMotionAllowedClosedLoopError(Constants.LauncherConstants.rpmTolerance, 0);

        //lowerLauncherWheels.follow(upperLauncherWheels, false);

        lowerVelocityController.setP(Constants.LauncherConstants.launcherP, 0);
        lowerVelocityController.setI(Constants.LauncherConstants.launcherI, 0);
        lowerVelocityController.setD(Constants.LauncherConstants.launcherD, 0);
        lowerVelocityController.setFF(Constants.LauncherConstants.launcherFF, 0);
        //lowerVelocityController.setSmartMotionAllowedClosedLoopError(Constants.LauncherConstants.rpmTolerance, 0);

        //noteHolderPositionController.setP(Constants.LauncherConstants.noteP, 0);
        //noteHolderPositionController.setI(Constants.LauncherConstants.noteI, 0);
        //noteHolderPositionController.setD(Constants.LauncherConstants.noteD, 0);
        //noteHolderPositionController.setFF(Constants.LauncherConstants.noteFF, 0);

        //set up state triggers
        noteInSerializerSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return noteInHolder.get();// & launcherState==LauncherState.SERIALIZE_NOTE;
            }
        };
        noteInSerializerTriggered = new Trigger(noteInSerializerSupplier);

        noteShotSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return noteInHolder.get() & launcherState==LauncherState.LAUNCHING;
            }
        };
        noteShotTriggered = new Trigger(noteShotSupplier);
    
        proximitySensor = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !noteInHolder.get(); //& launcherState==LauncherState.LAUNCHING;
            }
        };

        atRPMSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return desiredRPM <= lowerLauncherWheels.getEncoder().getVelocity();//MathUtil.isNear(desiredRPM, lowerLauncherWheels.getEncoder().getVelocity(), Constants.LauncherConstants.rpmTolerance);
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

        noteHolder.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
        
        upperLauncherWheels.burnFlash();
        lowerLauncherWheels.burnFlash();
        noteHolder.setIdleMode(IdleMode.kBrake);
        //noteHolder.burnFlash();
        //noteHolder.burnFlash();
    }

    public Trigger getNoteSerialized() {
        return noteInSerializerTriggered;
    }

    public Trigger getNoteShot() {
        return noteShotTriggered;
    }

    public BooleanSupplier getProximitySensor() {
        return proximitySensor;
    }

    public Trigger getLauncherAtRPM() {
        return atRPMTriggered;
    }

    public void setRPMSpeaker() {
         
        upperVelocityController.setReference(1200, CANSparkBase.ControlType.kVelocity);
        lowerVelocityController.setReference(1200, CANSparkBase.ControlType.kVelocity);
        desiredRPM = 1200;
    }

    public void setRPMAmp() {

        // upperVelocityController.setReference(40, CANSparkBase.ControlType.kVelocity);
        // lowerVelocityController.setReference(400, CANSparkBase.ControlType.kVelocity);
        // desiredRPM = 400;
        
         upperLauncherWheels.set(0.05);
         lowerLauncherWheels.set(0.25);
         desiredRPM = 50;
    }

    public void setRPMTrap(){

        upperVelocityController.setReference(370, CANSparkBase.ControlType.kVelocity);
        lowerVelocityController.setReference(390, CANSparkBase.ControlType.kVelocity);
        desiredRPM = 390;
        
        // upperLauncherWheels.set(0.55);
        //  lowerLauncherWheels.set(0.55);
        //  desiredRPM = 300;
    }

    public void setRPMShuttle(){
        upperVelocityController.setReference(600, CANSparkBase.ControlType.kVelocity);
        lowerVelocityController.setReference(600, CANSparkBase.ControlType.kVelocity);
        desiredRPM = 600;
    }

    public void setIdle() {
        upperVelocityController.setReference(0, CANSparkBase.ControlType.kVelocity);
        lowerVelocityController.setReference(0, CANSparkBase.ControlType.kVelocity);
        desiredRPM = 0;
    }

    public double getRPM() {
        return upperLauncherWheels.getEncoder().getVelocity();
    }

    public void setState(LauncherState launcherState) {

        this.launcherState = launcherState;
    }

    public void periodic() {
        
        // switch (launcherState) {

        //     case AIMING_AMP:
        //         upperVelocityController.setReference(10, CANSparkBase.ControlType.kVelocity);
        //         lowerVelocityController.setReference(300, CANSparkBase.ControlType.kVelocity);
        //         desiredRPM = 300;
        //         noteHolder.set(0);
        //         System.out.println("Aiming Amp!");
        //         break;
            
        //     case AIMING_SPEAKER:
                
        //         upperVelocityController.setReference(1000, CANSparkBase.ControlType.kVelocity);
        //         lowerVelocityController.setReference(1000, CANSparkBase.ControlType.kVelocity);
        //         desiredRPM = 1000;
        //        // noteHolderPositionController.setReference(0, ControlType.kVelocity);
        //         noteHolder.set(0);
        //         break;
            
        //     case LAUNCHING:
        //         //upperVelocityController.setReference(4200, ControlType.kVelocity);
        //         //lowerVelocityController.setReference(4200, ControlType.kVelocity);
        //         //Note, keep the speed constant here, don't update rpm value setpoint
        //        // noteHolderPositionController.setReference(Constants.LauncherConstants.launchingHolderSpeed, ControlType.kVelocity);
               
        //         noteHolder.set(-0.8);
                
        //         break;
            
        //     case SERIALIZE_NOTE:
                
        //         upperVelocityController.setReference(Constants.LauncherConstants.idleRPM, ControlType.kVelocity);
        //         lowerVelocityController.setReference(Constants.LauncherConstants.idleRPM, ControlType.kVelocity);
        //         //noteHolderPositionController.setReference(Constants.LauncherConstants.dumbHolderSpeed, ControlType.kVelocity);
        //          noteHolder.set(-0.8);
        //         break;

        //     case NOTE_IN_HOLDER:

        //         upperVelocityController.setReference(Constants.LauncherConstants.idleRPM, ControlType.kVelocity);
        //         lowerVelocityController.setReference(Constants.LauncherConstants.idleRPM, ControlType.kVelocity);
        //        // noteHolderPositionController.setReference(0, ControlType.kVelocity);
        //         noteHolder.set(0);
        //         break;
        //     case GATE_WHEEL:
        //         noteHolder.set(0);
        //         break;

        //     case IDLE_RPM:

        //         upperLauncherWheels.set(0);
        //         lowerLauncherWheels.set(0);
        //         //noteHolderPositionController.setReference(0, ControlType.kVelocity);
        //          noteHolder.set(0);
        //         break;

        //     case HOME:
        //     default:
                
        //         break;
        //}
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
