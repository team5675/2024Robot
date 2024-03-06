package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Climber extends SubsystemBase implements WiredSubsystem {
    
    static Climber instance;

    public enum ClimberState implements InnerWiredSubsystemState {
        HOME,
        LOCKED,
        EXTENDING,
        RETRACTING,
    } 

    ClimberState climberState;

    CANSparkMax winchMotor;
    SparkPIDController winchPID;

    Servo releaseServo;

    public Climber() {

        climberState = ClimberState.HOME;

        winchMotor = new CANSparkMax(Constants.ClimberConstants.climberMotorID, MotorType.kBrushless);
        releaseServo = new Servo(Constants.ClimberConstants.servoID);

        // winchPID = winchMotor.getPIDController();

        // winchPID.setP(Constants.ClimberConstants.climbP);
        // winchPID.setI(Constants.ClimberConstants.climbI);
        // winchPID.setD(Constants.ClimberConstants.climbD);
    }

    public Trigger getClimbComplete() {

        BooleanSupplier test = new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                //get complete climb conditions
                return true;
            }
        };

        return new Trigger(test);
    }

    public void periodic() {

        switch (climberState) {
            case LOCKED:
                releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeClosed);
                winchMotor.set(0);
                releaseServo.setAngle(40);
                break;
            
            case RETRACTING:
                releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeOpen);
                winchMotor.set(1);
                break;
            case EXTENDING:
                releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeOpen);
                winchMotor.set(-0.6);
                break;
            case HOME:
            default:
                winchMotor.set(0);
                releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeOpen);
                 break;
        }
    }

    public void setState(ClimberState state) {
        climberState = state;
    }

    @Override
    public InnerWiredSubsystemState getState() {
        return climberState;
    }

    @Override
    public void reportData() {
         SmartDashboard.putString("Climber State", climberState.toString());
    }


    public static Climber getInstance() {
        if(instance == null)
            instance = new Climber();

        return instance;
    }


    
}
