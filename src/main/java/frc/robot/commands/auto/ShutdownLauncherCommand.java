package frc.robot.commands.auto;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class ShutdownLauncherCommand extends Command {
 
    public void intialize() {

        addRequirements(Launcher.getInstance());
    }

    public void execute() {
        
        Launcher.getInstance().upperVelocityController.setReference(0, ControlType.kVelocity);
                
         Launcher.getInstance().lowerVelocityController.setReference(0, ControlType.kVelocity);
         Launcher.getInstance().noteHolder.set(0);
    }

    public boolean isFinished() {
        
        return true; 
        
    }

    public void end() {
      
    }
}
