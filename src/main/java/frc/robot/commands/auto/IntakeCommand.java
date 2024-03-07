package frc.robot.commands.auto;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.RobotState.Event;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class IntakeCommand extends Command {

   
 public IntakeCommand(){
    System.out.println("Constructor Intake");
    
    //RobotState.getInstance().setEvent(Event.INTAKE_REQUEST);
    
 }
    public void intialize() {
        
       //RobotState.getInstance().setEvent(Event.INTAKE_REQUEST);
    }

    public void execute() {
         RobotState.getInstance().setEvent(Event.INTAKE_REQUEST);
     
        //Put event in here?
    }

    public boolean isFinished() {
        
        //return when note in launcher
        return Launcher.getInstance().noteInHolder.get();
    }

    public void end() {
    Intake.getInstance().intakeMotor.set(0);
    Launcher.getInstance().noteHolder.set(0);
    Launcher.getInstance().upperVelocityController.setReference(0, CANSparkBase.ControlType.kVelocity);
    Launcher.getInstance().lowerVelocityController.setReference(0, CANSparkBase.ControlType.kVelocity);
    
    }
}

