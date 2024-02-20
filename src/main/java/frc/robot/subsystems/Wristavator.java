package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Wristavator extends SubsystemBase implements WiredSubsystem {

    static Wristavator instance; 

    CANSparkBase wristMotor;
    CANSparkBase elevatorMotor;

    SparkPIDController wristPID;
    SparkPIDController elevatorPID;

    ArmFeedforward wristFeedforward;
    ElevatorFeedforward elevatorFeedforward;

    TrapezoidProfile wristProfile;
    TrapezoidProfile elevatorProfile;

    TrapezoidProfile.State setpointWristState;
    TrapezoidProfile.State setpointElevatorState;

    TrapezoidProfile.State goalWristState;
    TrapezoidProfile.State goalElevatorState;

    BooleanSupplier isAtDesiredHeight;
    BooleanSupplier isAtDesiredAngle;

    Trigger heightTrigger;
    Trigger angleTrigger;

    double desiredHeightMeters;
    double desiredAngleDegrees;

    double lastTime;
    double nowTime;

    ShuffleboardTab wristavatorTab;

    public enum WristavatorState implements InnerWiredSubsystemState {
        HOME,
        INTAKING,
        LAUNCHING_SPEAKER,
        LAUNCHING_AMP,
        LAUNCHING_TRAP,
        STOWED
    }

    WristavatorState wristavatorState;

    public Wristavator() {

        wristavatorState = WristavatorState.HOME;

        wristMotor = new CANSparkFlex(Constants.WristavatorConstants.wristID, MotorType.kBrushless);
        elevatorMotor = new CANSparkFlex(Constants.WristavatorConstants.elevatorID, MotorType.kBrushless);

        wristPID = wristMotor.getPIDController();
        elevatorPID = elevatorMotor.getPIDController();

        wristPID.setP(Constants.WristavatorConstants.wristP);
        wristPID.setI(Constants.WristavatorConstants.wristI);
        wristPID.setD(Constants.WristavatorConstants.wristD);

        elevatorPID.setP(Constants.WristavatorConstants.elevatorP);
        elevatorPID.setI(Constants.WristavatorConstants.elevatorI);
        elevatorPID.setD(Constants.WristavatorConstants.elevatorD);

        wristPID.setFeedbackDevice(wristMotor.getAbsoluteEncoder(Type.kDutyCycle));
        elevatorPID.setFeedbackDevice(elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle));

        wristFeedforward = new ArmFeedforward(Constants.WristavatorConstants.wristKS, 
                                              Constants.WristavatorConstants.wristKG, 
                                              Constants.WristavatorConstants.wristKV, 0);
        
        elevatorFeedforward = new ElevatorFeedforward(Constants.WristavatorConstants.elevatorKS, 
                                                      Constants.WristavatorConstants.elevatorKG, 
                                                      Constants.WristavatorConstants.elevatorKV, 0);

        wristProfile = new TrapezoidProfile(Constants.WristavatorConstants.wristProfileConstraints);
        elevatorProfile = new TrapezoidProfile(Constants.WristavatorConstants.elevatorProfileConstraints);

        setpointWristState = new TrapezoidProfile.State();
        setpointElevatorState = new TrapezoidProfile.State(); 
        
        goalWristState = new TrapezoidProfile.State();
        goalElevatorState = new TrapezoidProfile.State();

        desiredHeightMeters = 0;
        desiredAngleDegrees = 0;

        isAtDesiredHeight = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return MathUtil.isNear(desiredHeightMeters, 
                    elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition(), Constants.WristavatorConstants.elevatorTolerance);
            }
        };

        isAtDesiredAngle = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return MathUtil.isNear(desiredAngleDegrees, 
                    wristMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition(), Constants.WristavatorConstants.wristTolerance);
            }
        };

        //smartdashboard data tab
        wristavatorTab = Shuffleboard.getTab("Wristavator");
        wristavatorTab.addDouble("Wrist Angle", () -> wristMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
        wristavatorTab.addDouble("Elevator Height", () -> elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
        //setpoint pos for both
        //setpoint velocity for both

        wristMotor.burnFlash();
        elevatorMotor.burnFlash();
    }

    public void setState(WristavatorState wristavatorState) {

        this.wristavatorState = wristavatorState;
    }

    public void periodic() {

        switch (wristavatorState) {
            case INTAKING:

                setpointWristState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorIntakePose.getAngle().getDegrees(), 0);
                setpointElevatorState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorIntakePose.getNorm(), 0);
                break;
            
            case LAUNCHING_SPEAKER:

                setpointWristState = new TrapezoidProfile.State(Limelight.getInstance().getPoseLauncherToSpeaker().getRotation().getY(), 0);
                setpointElevatorState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorHomePose.getNorm(), 0);
                break;

            case LAUNCHING_AMP:

                setpointWristState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorAmpPose.getAngle().getDegrees(), 0);
                setpointElevatorState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorAmpPose.getNorm(), 0);
                break;

            case LAUNCHING_TRAP:

                setpointWristState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorTrapPose.getAngle().getDegrees(), 0);
                setpointElevatorState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorTrapPose.getNorm(), 0);
                break;
        
            default:

                setpointWristState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorHomePose.getAngle().getDegrees(), 0);
                setpointElevatorState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorHomePose.getNorm(), 0);
                break;
        }

        lastTime = nowTime;
        nowTime = java.lang.System.currentTimeMillis();

        //always update position of these
        setpointWristState = wristProfile.calculate(nowTime-lastTime, setpointWristState, goalWristState);
        setpointElevatorState = elevatorProfile.calculate(nowTime-lastTime, setpointElevatorState, goalElevatorState);

        wristPID.setReference(setpointWristState.position, ControlType.kPosition, 0, 
            wristFeedforward.calculate(setpointWristState.position, setpointWristState.velocity));

        elevatorPID.setReference(setpointElevatorState.position, ControlType.kPosition, 0, 
            elevatorFeedforward.calculate(setpointElevatorState.position, setpointElevatorState.velocity));
    }

    @Override
    public InnerWiredSubsystemState getState() {
        return wristavatorState;
    }

    @Override
    public void reportData() {
        SmartDashboard.putString("WristavatorState", wristavatorState.toString());
    }
    
    public static Wristavator getInstance() {
        if (instance == null) {
            instance = new Wristavator();
        }
        return instance;
    }
}
