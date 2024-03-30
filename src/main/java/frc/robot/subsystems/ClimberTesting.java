package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.WiredSubsystem.InnerWiredSubsystemState;

public class ClimberTesting extends SubsystemBase implements WiredSubsystem {
    
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

    public ClimberTesting() {

        climberState = ClimberState.HOME;

        winchMotor = new CANSparkMax(Constants.ClimberConstants.climberMotorID, MotorType.kBrushless);
        releaseServo = new PWM(Constants.ClimberConstants.servoID);
    }
    public void setState(ClimberState state) {
        climberState = state;
}
public void unlockClimber() {
    releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeOpen);
}

public void lockClimber() {
    releaseServo.setPulseTimeMicroseconds(Constants.ClimberConstants.latchPulseTimeClosed);
}

public void raiseClimber() {
    double value;
    setReference​(value, CANSparkBase.ControlType ctrl)
    winchMotor.getPIDController();
}

public void stopClimber() {
    winchMotor.set(0);
}
@Override
public InnerWiredSubsystemState getState() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getState'");
}
@Override
public void reportData() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reportData'");
}

}