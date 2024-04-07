package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class NoNoteCommand extends Command {

   
 public NoNoteCommand(){
    
    
 }
    @Override
    public void initialize() {
     
       addRequirements(Intake.getInstance(), Launcher.getInstance());
    }

    @Override
    public void execute() {
     
    }

    @Override
    public boolean isFinished() {
        if(!Launcher.getInstance().noteInHolder.get()){
            return true;
        } else
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().intakeMotor.set(0);
        Launcher.getInstance().noteHolder.set(0);
    }
}

