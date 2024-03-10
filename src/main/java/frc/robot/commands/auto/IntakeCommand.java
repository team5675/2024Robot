package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class IntakeCommand extends Command {

   
 public IntakeCommand(){
    
    
 }
    @Override
    public void initialize() {
     
       addRequirements(Intake.getInstance(), Launcher.getInstance());
    }

    @Override
    public void execute() {
        Intake.getInstance().intakeMotor.set(-0.9);
        Launcher.getInstance().noteHolder.set(-0.8);
     
    }

    @Override
    public boolean isFinished() {
        
        //return when note in launcher
        return !Launcher.getInstance().noteInHolder.get();
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().intakeMotor.set(0);
        Launcher.getInstance().noteHolder.set(0);
    }
}

