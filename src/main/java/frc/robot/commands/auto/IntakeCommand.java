package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.RobotState.Event;
import frc.robot.RobotState.State;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends InstantCommand {

   
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
   
     
        //Put event in here?
    }

    public boolean isFinished() {
        System.out.println("Auto Intake Finished");
        return Intake.getInstance().getIntakeProxTriggered().getAsBoolean();
    }

    public void end() {;
         //TODO Add: Intake.getInstance().setState(Intake.IntakeState.HOME);
    }
    
    }

