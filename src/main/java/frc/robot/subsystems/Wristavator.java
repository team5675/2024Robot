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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Wristavator implements WiredSubsystem {

    Wristavator instance; 

    CANSparkBase wristMotor;
    CANSparkBase elevatorMotor;

    SparkPIDController wristPID;
    SparkPIDController elevatorPID;

    ArmFeedforward wristFeedforward;
    ElevatorFeedforward elevatorFeedforward;

    BooleanSupplier isAtDesiredHeight;
    BooleanSupplier isAtDesiredAngle;

    Trigger heightTrigger;
    Trigger angleTrigger;

    double desiredHeightMeters;
    double desiredAngleDegrees;

    double wristFeedforwardDouble;
    double elevatorFeedforwardDouble;

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
        elevatorPID.setFeedbackDevice(wristMotor.getAbsoluteEncoder(Type.kDutyCycle));

        wristFeedforward = new ArmFeedforward(Constants.WristavatorConstants.wristKS, 
                                              Constants.WristavatorConstants.wristKG, 
                                              Constants.WristavatorConstants.wristKV, 0);
        
        elevatorFeedforward = new ElevatorFeedforward(Constants.WristavatorConstants.elevatorKS, 
                                                      Constants.WristavatorConstants.elevatorKG, 
                                                      Constants.WristavatorConstants.elevatorKV, 0);

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
    }

    public void setAngle(double desiredAngleDegrees) {

        wristFeedforwardDouble = wristFeedforward.calculate(desiredAngleDegrees, 0);

        wristPID.setReference(desiredAngleDegrees, ControlType.kPosition, 0, wristFeedforwardDouble);
    }

    public void setHeight(double desiredHeightMeters) {

        //TODO: More scuffed than my bare ass on a pine tree
        elevatorFeedforwardDouble = elevatorFeedforward.calculate(elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity(), 0);

        elevatorPID.setReference(desiredHeightMeters, ControlType.kSmartMotion, 0, elevatorFeedforwardDouble);
    }

    public void periodic() {

        switch (wristavatorState) {
            case INTAKING:

                setAngle(Constants.WristavatorConstants.wristavatorIntakePose.getNorm());
                setHeight(Constants.WristavatorConstants.wristavatorIntakePose.getAngle().getDegrees());
                
                break;
            
            case LAUNCHING_SPEAKER:

            //TODO: Add Variable height and angle to wristavator;

                break;

            case LAUNCHING_AMP:

                setAngle(Constants.WristavatorConstants.wristavatorAmpPose.getNorm());
                setHeight(Constants.WristavatorConstants.wristavatorAmpPose.getAngle().getDegrees());

                break;

            case LAUNCHING_TRAP:

                setAngle(Constants.WristavatorConstants.wristavatorTrapPose.getNorm());
                setHeight(Constants.WristavatorConstants.wristavatorTrapPose.getAngle().getDegrees());

                break;
        
            default:

                setAngle(Constants.WristavatorConstants.wristavatorHomePose.getNorm());
                setHeight(Constants.WristavatorConstants.wristavatorHomePose.getAngle().getDegrees());

                break;
        }
    }

    @Override
    public InnerWiredSubsystemState getState() {
        throw new UnsupportedOperationException("Unimplemented method 'getState'");
    }

    @Override
    public void reportData() {
        throw new UnsupportedOperationException("Unimplemented method 'reportData'");
    }
    
}
