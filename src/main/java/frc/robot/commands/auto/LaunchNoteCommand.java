package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.RobotState.Event;
import frc.robot.subsystems.Launcher;

public class LaunchNoteCommand extends Command {
 
    public void intialize() {

        RobotState.getInstance().setEvent(Event.LAUNCH_SPEAKER_REQUEST);
    }

    public void execute() {
        RobotState.getInstance().setEvent(Event.LAUNCH_SPEAKER_REQUEST);
        //  Launcher.getInstance().upperVelocityController.setReference(3000, ControlType.kVelocity);
                
        // Launcher.getInstance().lowerVelocityController.setReference(3000, ControlType.kVelocity);
        // Launcher.getInstance().noteHolder.set(-0.1);
    }

    public boolean isFinished() {
        return Launcher.getInstance().noteInHolder.get();
    }

    public void end() {
    }
}
