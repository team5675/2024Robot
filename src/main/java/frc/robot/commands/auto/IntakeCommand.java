package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class IntakeCommand extends Command {

   
 public IntakeCommand(){
    System.out.println("Constructor Intake");
    
    //RobotState.getInstance().setEvent(Event.INTAKE_REQUEST);
    
 }
    public void intialize() {
        System.out.println("Auto Intake init");
       //RobotState.getInstance().setEvent(Event.INTAKE_REQUEST);
    }

    public void execute() {
        Intake.getInstance().intakeMotor.set(-0.7);
        Launcher.getInstance().noteHolder.set(-0.8);
     
        //Put event in here?
    }

    public boolean isFinished() {
        System.out.println("Auto Intake Finished");
        //return when note in launcher
        return !Launcher.getInstance().noteInHolder.get();
    }

    public void end() {
        Intake.getInstance().intakeMotor.set(0);
        Launcher.getInstance().noteHolder.set(0);
    }
    
    }

