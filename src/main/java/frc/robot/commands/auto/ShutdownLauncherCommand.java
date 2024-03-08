package frc.robot.commands.auto;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class ShutdownLauncherCommand extends InstantCommand {
 
    public void intialize() {

        addRequirements(Launcher.getInstance(), Intake.getInstance());
    }

    public void execute() {
        
        Launcher.getInstance().upperVelocityController.setReference(0, ControlType.kVelocity);
        Launcher.getInstance().lowerVelocityController.setReference(0, ControlType.kVelocity);
        Launcher.getInstance().noteHolder.set(0);
        Intake.getInstance().intakeMotor.set(0);

    }

    public boolean isFinished() {
        
        return true; 
        
    }

    public void end() {
      
    }
}
