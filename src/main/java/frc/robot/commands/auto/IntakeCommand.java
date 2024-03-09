package frc.robot.commands.auto;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.RobotState.Event;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class IntakeCommand extends Command {

   
 public IntakeCommand(){
    System.out.println("Constructor Intake");
    
    //RobotState.getInstance().setEvent(Event.INTAKE_REQUEST);
    
 }
    @Override
    public void initialize() {
        
       //RobotState.getInstance().setEvent(Event.INTAKE_REQUEST);
       addRequirements(Intake.getInstance(), Launcher.getInstance());
    }

    @Override
    public void execute() {
         Intake.getInstance().intakeMotor.set(-0.9);
         Launcher.getInstance().noteHolder.set(-0.8);
     
        //Put event in here?
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

