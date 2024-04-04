package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;

// This command aligns the robot based on feedback from the limelight camera
public class ForwardNudge extends Command {
   
    private Swerve drive = Swerve.getInstance();
    

    @Override
    public void initialize() {
        // Nothing to initialize
        addRequirements(Swerve.getInstance());
    }

    @Override
    public void execute() {
        double threeInches = 0.0762;
        Translation2d ForwardNudge = new Translation2d(threeInches,0);
          
          drive.drive(ForwardNudge,0.0,false);
            }    

    @Override
    public void end(boolean interrupted) {
        
        // Command ends, stop the movement
        // You may need to add additional logic here based on your requirements
       
    }
}
