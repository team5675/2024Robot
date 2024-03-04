package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotState.Event;
import frc.robot.subsystems.Launcher;

public class LaunchNoteCommand extends Command {
 
    public void intialize() {

        //RobotState.getInstance().setEvent(Event.LAUNCH_SPEAKER_REQUEST);
    }

    public void execute() {
        Commands.runOnce(
        () -> RobotState.getInstance().setEvent(Event.LAUNCH_SPEAKER_REQUEST));
    }

    public boolean isFinished() {
        return Launcher.getInstance().getNoteShot().getAsBoolean();
    }

    public void end() {}
}
