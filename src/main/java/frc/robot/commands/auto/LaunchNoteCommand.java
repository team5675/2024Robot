package frc.robot.commands.auto;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class LaunchNoteCommand extends Command {
 
    public void intialize() {
        addRequirements(Launcher.getInstance());
        
    }

    public void execute() {
        
        Launcher.getInstance().upperVelocityController.setReference(1000, ControlType.kVelocity);     
        Launcher.getInstance().lowerVelocityController.setReference(1000, ControlType.kVelocity);

        if(Launcher.getInstance().upperLauncherWheels.getEncoder().getVelocity() >= 1000) {
            Launcher.getInstance().noteHolder.set(-0.8);
          } else {
            Launcher.getInstance().noteHolder.set(0);
          }
    }

    public boolean isFinished() {
        
        return Launcher.getInstance().upperLauncherWheels.getEncoder().getVelocity() >= 1000;
        
    }

    public void end() {

        Launcher.getInstance().upperVelocityController.setReference(0, ControlType.kVelocity);     
        Launcher.getInstance().lowerVelocityController.setReference(0, ControlType.kVelocity);
        Launcher.getInstance().noteHolder.set(0);
    }
}
