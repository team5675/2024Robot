package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
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

    PWM releaseServo;

    public Climber() {

        climberState = ClimberState.HOME;

        winchMotor = new CANSparkMax(Constants.ClimberConstants.climberMotorID, MotorType.kBrushless);
        releaseServo = new PWM(Constants.ClimberConstants.servoID);

        releaseServo.setBoundsMicroseconds(2500, 50, 1500, 50, 500);

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

    public void unlockClimber() {
        releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeOpen);
    }

    public void lockClimber() {
        releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeClosed);
    }

    public void raiseClimber() {
        winchMotor.set(1);
    }

    public void lowerClimber() {
        winchMotor.set(-0.6);
    }

    public void stopClimber() {
        winchMotor.set(0);
    }

    public void periodic() {

        // switch (climberState) {
        //     case LOCKED:
        //         releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeClosed);
        //         winchMotor.set(0);
        //         break;
            
        //     case RETRACTING:
        //         releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeOpen);
        //         winchMotor.set(1);
        //         break;
        //     case EXTENDING:
        //         releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeOpen);
        //         winchMotor.set(-0.6);
        //         break;
        //     case HOME:
        //     default:
        //         winchMotor.set(0);
        //         releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeOpen);
        //         break;
        // }
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
