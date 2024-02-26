package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.RobotState.Event;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
 
    public void intialize() {

        RobotState.getInstance().setEvent(Event.INTAKE_REQUEST);
    }

    public void execute() {}

    public boolean isFinished() {
        return Intake.getInstance().getIntakeProxTriggered().getAsBoolean();
    }

    public void end() {}
}
