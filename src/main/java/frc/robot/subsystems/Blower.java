package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

public class Blower extends SubsystemBase implements WiredSubsystem {
    
    public static Blower instance;

    ShuffleboardTab BlowerTab;
    
    double BlowerRPM;

    public CANSparkMax blowerMotorAmp;  
    public CANSparkMax blowerMotorTrapLeft;
    public CANSparkMax blowerMotorTrapRight;  

    public enum BlowerState implements InnerWiredSubsystemState {
        HOME,
        BLOWING
    }
    
    BlowerState BlowerState;

    public Blower() {

        blowerMotorAmp = new CANSparkMax(Constants.BlowerConstants.blowerMotorAmpID,  MotorType.kBrushed);
        blowerMotorTrapLeft = new CANSparkMax(Constants.BlowerConstants.leftBlowerMotorTrapID,  MotorType.kBrushed);
        blowerMotorTrapRight = new CANSparkMax(Constants.BlowerConstants.rightBlowerMotorTrapID,  MotorType.kBrushed);
        BlowerTab = Shuffleboard.getTab("Blower");
        BlowerTab.addDouble("Blower RPM", () -> BlowerRPM);
        blowerMotorTrapLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        blowerMotorTrapRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        blowerMotorAmp.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        blowerMotorAmp.burnFlash();
        blowerMotorTrapLeft.burnFlash();
        blowerMotorTrapRight.burnFlash();
    }

    public void periodic() {

        
    }

    /**
     * Get the Trigger for the intake prox sensor
     * @return intake prox state
     */
    
    /**
     * Speed to run intake at
     * @param rpm desired intake rpm (-5676 to 5676)
     */


    public static Blower getInstance() {
        if (instance == null) 
            instance = new Blower();

        return instance;
    }

    @Override
    public void reportData() {
    }

    @Override
    public InnerWiredSubsystemState getState() {
        return BlowerState;
    }
}