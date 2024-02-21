package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Climber extends SubsystemBase implements WiredSubsystem {
    
    static Climber instance;

    public enum ClimberState implements InnerWiredSubsystemState {
        HOME,
        UNLATCHED,
        EXTENDED,
        RATCHETING,
    } 

    ClimberState climberState;

    CANSparkMax winchMotor;
    SparkPIDController winchPID;

    Servo releaseServo;

    public Climber() {

        climberState = ClimberState.HOME;

        winchMotor = new CANSparkMax(Constants.ClimberConstants.climberMotorID, MotorType.kBrushless);
        releaseServo = new Servo(Constants.ClimberConstants.servoID);

        winchPID = winchMotor.getPIDController();

        winchPID.setP(Constants.ClimberConstants.climbP);
        winchPID.setI(Constants.ClimberConstants.climbI);
        winchPID.setD(Constants.ClimberConstants.climbD);
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
            case UNLATCHED:
                releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeOpen);
                break;
            
            case EXTENDED:
                winchPID.setReference(Constants.ClimberConstants.climbExtended.getDegrees(), ControlType.kPosition);
        
            case HOME:
            default:

                releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeClosed);
                winchPID.setReference(Constants.ClimberConstants.climbRetracted.getDegrees(), ControlType.kPosition);
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reportData'");
    }


    public static Climber getInstance() {
        if(instance == null)
            instance = new Climber();

        return instance;
    }


    
}
