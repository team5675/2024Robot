package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Intake extends SubsystemBase implements WiredSubsystem {
    
    public static Intake instance;

    ShuffleboardTab intakeTab;

    DigitalInput intakeProx;
    BooleanSupplier intakeProxSupplier;
    Trigger intakeProxTriggered;

    double intakeRPM;

    public enum IntakeState implements InnerWiredSubsystemState {
        HOME,
        DEPLOYED,
        INTAKING,
        OUTTAKING,
    }
    
    IntakeState intakeState;
    boolean isDeployed;
    boolean isRetracted;
    boolean isIntaking;
    boolean isOuttaking;
    boolean isHome;

    public Intake() {

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
        intakeTab.addBoolean("Intake Deployed", () -> isDeployed && !isRetracted);

        isDeployed = false;
        isRetracted = false;
        isIntaking = false;
        isOuttaking = false;
        isHome = false;
    }

    public void setState(IntakeState intakeState) {
        
        this.intakeState = (IntakeState)intakeState;
    }

    public void periodic() {

        switch (intakeState) {
            case INTAKING:

                deployIntake();
                setSpeed(Constants.IntakeConstants.IntakeSpeedRPM);
                break;

            case OUTTAKING:

                deployIntake();
                setSpeed(Constants.IntakeConstants.OuttakeSpeedRPM);
                break;

            case DEPLOYED:

                deployIntake();
                setSpeed(0);
                break;

            case HOME:
            default:
                retractIntake();
                setSpeed(0);
                break;
        }

        if(intakeState == IntakeState.HOME)
            isHome = true;
        else 
            isHome = false;
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

    /**
     * Deploys Intake to collect Note
     */
    private void deployIntake() {

        isDeployed = true;
        isRetracted = false;
    }

    /**
     * Retracts intake back into frame perimeter
     */
    private void retractIntake() {

        isDeployed = false;
        isRetracted = true;
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
