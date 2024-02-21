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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
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

    DigitalInput wristLimitSwitch;
    DigitalInput elevatorLimitSwitch;

    ArmFeedforward wristFeedforward;
    ElevatorFeedforward elevatorFeedforward;

    TrapezoidProfile wristProfile;
    TrapezoidProfile elevatorProfile;

    TrapezoidProfile.State currentWristState;
    TrapezoidProfile.State currentElevatorState;

    TrapezoidProfile.State goalWristState;
    TrapezoidProfile.State goalElevatorState;

    BooleanSupplier isAtDesiredHeight;
    BooleanSupplier isAtDesiredAngle;
    BooleanSupplier isAimingComplete;

    Trigger heightTrigger;
    Trigger angleTrigger;
    Trigger aimedTrigger;
    Trigger wristAtZeroTrigger;
    Trigger elevatorAtZeroTrigger;

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

        wristLimitSwitch = new DigitalInput(Constants.WristavatorConstants.wristLimitSwitchId);
        elevatorLimitSwitch = new DigitalInput(Constants.WristavatorConstants.elevatorLimitSwitchID);

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

        wristMotor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(Constants.WristavatorConstants.wristPositionOffset);
        elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(Constants.WristavatorConstants.elevatorPositionOffset);

        wristFeedforward = new ArmFeedforward(Constants.WristavatorConstants.wristKS, 
                                              Constants.WristavatorConstants.wristKG, 
                                              Constants.WristavatorConstants.wristKV, 0);
        
        elevatorFeedforward = new ElevatorFeedforward(Constants.WristavatorConstants.elevatorKS, 
                                                      Constants.WristavatorConstants.elevatorKG, 
                                                      Constants.WristavatorConstants.elevatorKV, 0);

        wristProfile = new TrapezoidProfile(Constants.WristavatorConstants.wristProfileConstraints);
        elevatorProfile = new TrapezoidProfile(Constants.WristavatorConstants.elevatorProfileConstraints);

        currentWristState = new TrapezoidProfile.State();
        currentElevatorState = new TrapezoidProfile.State(); 
        
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

        isAimingComplete = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isAtDesiredAngle.getAsBoolean() && isAtDesiredAngle.getAsBoolean();
            }
        };
        aimedTrigger = new Trigger(isAimingComplete);

        wristAtZeroTrigger = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return wristLimitSwitch.get();
            }
        });

        elevatorAtZeroTrigger = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return elevatorLimitSwitch.get();
            }
        });

        //smartdashboard data tab
        wristavatorTab = Shuffleboard.getTab("Wristavator");
        wristavatorTab.addDouble("Wrist Angle", () -> wristMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
        wristavatorTab.addDouble("Elevator Height", () -> elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
        //setpoint pos for both
        //setpoint velocity for both

        wristMotor.burnFlash();
        elevatorMotor.burnFlash();
    }

    public Trigger getAimingComplete() {
        return aimedTrigger;
    }

    public void setState(WristavatorState wristavatorState) {

        this.wristavatorState = wristavatorState;
    }

    public double getElevatorHeight() {
        return elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }

    public Rotation2d getWristAngle() {
        return Rotation2d.fromDegrees(wristMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    }

    //Height of elevator in meters, 0 is the carpet
    //Please initialize to the {@Link Constants.LauncherConstants.elevatorZeroOffset}
    public void setElevatorZeroHeight(double height) {
        elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).setZeroOffset(height);
    }

    public void setWristZeroAngle(Rotation2d angle) {
        wristMotor.getAbsoluteEncoder(Type.kDutyCycle).setZeroOffset(angle.getDegrees());
    }

    public void periodic() {

        switch (wristavatorState) {
            case INTAKING:

                goalWristState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorIntakePose.getAngle().getDegrees(), 0);
                goalElevatorState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorIntakePose.getNorm(), 0);
                break;
            
            case LAUNCHING_SPEAKER:

                goalWristState = new TrapezoidProfile.State(Limelight.getInstance().getTranslationLauncherToSpeaker().getAngle().getDegrees(), 0);
                goalElevatorState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorHomePose.getNorm(), 0);
                break;

            case LAUNCHING_AMP:

                goalWristState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorAmpPose.getAngle().getDegrees(), 0);
                goalElevatorState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorAmpPose.getNorm(), 0);
                break;

            case LAUNCHING_TRAP:

                goalWristState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorTrapPose.getAngle().getDegrees(), 0);
                goalElevatorState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorTrapPose.getNorm(), 0);
                break;
    
            default:

                goalWristState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorHomePose.getAngle().getDegrees(), 0);
                goalElevatorState = new TrapezoidProfile.State(Constants.WristavatorConstants.wristavatorHomePose.getNorm(), 0);
                break;
        }

        lastTime = nowTime;
        nowTime = java.lang.System.currentTimeMillis();

        //always update position of these
        currentWristState = wristProfile.calculate(nowTime-lastTime, currentWristState, goalWristState);
        currentElevatorState = elevatorProfile.calculate(nowTime-lastTime, currentElevatorState, goalElevatorState);

        wristPID.setReference(currentWristState.position, ControlType.kPosition, 0, 
            wristFeedforward.calculate(goalWristState.position, goalWristState.velocity));

        elevatorPID.setReference(currentElevatorState.position, ControlType.kPosition, 0, 
            elevatorFeedforward.calculate(goalElevatorState.position, goalElevatorState.velocity));
    }

    /**
     * Returns the origin to launcher mouth vector in the <b>XZ PLANE</br>
     * Use for speaker aiming calculations
     * @return origin to launcher mouth vector
     */
    public Translation2d getOriginToLauncherMouthTranslationXZ() {

        Translation2d robotPositionXZ = new Translation2d(Swerve.getInstance().getRobotPose().getX(), 0);
        Translation2d launcherPositionXZ = new Translation2d(getElevatorHeight(), getWristAngle());

        return robotPositionXZ.plus(launcherPositionXZ);
    }

    /**
     * Returns the origin to launcher mouth vector in the <b>YZ PLANE</br>
     * Use for amp aiming calculations
     * @return origin to launcher mouth vector
     */
    public Translation2d getOriginToLauncherMouthTranslationYZ() {

        Translation2d robotPositionYZ = new Translation2d(Swerve.getInstance().getRobotPose().getY(), 0);
        Translation2d launcherPositionYZ = new Translation2d(getElevatorHeight(), getWristAngle());

        return robotPositionYZ.plus(launcherPositionYZ);
    }

    public Trigger getWristZeroTrigger() {
        return wristAtZeroTrigger;
    }

    public Trigger getElevatorZeroTrigger() {
        return elevatorAtZeroTrigger;
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
