package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class LEDCommand extends Command {

   
 public LEDCommand(){
     
 }
     @Override
     public void initialize() {
        addRequirements(LEDs.getInstance());
     }

     @Override
     public void execute() {
        LEDs.getInstance().setOrange();
      
     }

    @Override
    public boolean isFinished() {
        
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}

