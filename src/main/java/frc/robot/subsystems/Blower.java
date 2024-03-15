package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Blower extends SubsystemBase implements WiredSubsystem {
    
    public static Blower instance;

    ShuffleboardTab BlowerTab;
    
    double BlowerRPM;

    public CANSparkMax blowerMotorAmp;  
    public CANSparkMax blowerMotorTrap; 
    public enum BlowerState implements InnerWiredSubsystemState {
        HOME,
        BLOWING
    }
    
    BlowerState BlowerState;

    public Blower() {

        blowerMotorAmp = new CANSparkMax(Constants.BlowerConstants.blowerMotorID,  MotorType.kBrushed);
        blowerMotorTrap = new CANSparkMax(Constants.BlowerConstants.blowerMotorTrapID,  MotorType.kBrushed);
        BlowerTab = Shuffleboard.getTab("Blower");
        BlowerTab.addDouble("Blower RPM", () -> BlowerRPM);
        blowerMotorAmp.burnFlash();
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