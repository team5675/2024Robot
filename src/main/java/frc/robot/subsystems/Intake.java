package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

public class Intake extends SubsystemBase implements WiredSubsystem {
    
    public static Intake instance;

    ShuffleboardTab intakeTab;

    DigitalInput intakeProx;
    BooleanSupplier intakeProxSupplier;
    Trigger intakeProxTriggered;

    double intakeRPM;

    CANSparkMax intakeMotor;    

    public enum IntakeState implements InnerWiredSubsystemState {
        HOME,
        INTAKING,
        OUTTAKING, //SemiColon??
    }
    
    IntakeState intakeState;

    public Intake() {

        intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID,  MotorType.kBrushless);
        intakeMotor.getPIDController().setP(0.002);
        intakeProx = new DigitalInput(Constants.IntakeConstants.IntakeProxPort);

        intakeProxSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return intakeProx.get();
            }
        };

        intakeProxTriggered = new Trigger(intakeProxSupplier);

        intakeState = IntakeState.HOME;

        intakeTab = Shuffleboard.getTab("Intake");
        intakeTab.addDouble("Intake RPM", () -> intakeRPM);
    }

    public void setState(IntakeState intakeState) {
        
        this.intakeState = (IntakeState)intakeState;
    }

    public void periodic() {

        switch (intakeState) {
            case INTAKING:

            intakeMotor.getPIDController().setReference(Constants.IntakeConstants.IntakeSpeedRPM, ControlType.kSmartVelocity);
                break;

            case OUTTAKING:

            intakeMotor.getPIDController().setReference(Constants.IntakeConstants.OuttakeSpeedRPM, ControlType.kSmartVelocity);
                break;

            case HOME:
            default:

            intakeMotor.getPIDController().setReference(0, ControlType.kSmartVelocity);
                break;
        }
    }

    /**
     * Get the Trigger for the intake prox sensor
     * @return intake prox state
     */
    public Trigger getIntakeProxTriggered() {
        return intakeProxTriggered;
    }

    /**
     * Speed to run intake at
     * @param rpm desired intake rpm (-5676 to 5676)
     */
    private void setSpeed(double rpm) {

        intakeRPM = rpm;
        
    }

    public IntakeState getState() {
        return intakeState;
    }


    public static Intake getInstance() {
        if (instance == null) 
            instance = new Intake();

        return instance;
    }

    @Override
    public void reportData() {
        
        
        SmartDashboard.putString("Intake State", intakeState.toString());
    }
}
