package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class LimelightBlinkCommand extends Command {

    CommandXboxController driverController = RobotContainer.driverController;
    Timer timer = new Timer();

   public LimelightBlinkCommand() {
    
   }

   public void initialize() {
   }

   public void execute() {
    
    CommandXboxController driverController = RobotContainer.driverController;
    Timer timer = new Timer();
    timer.reset();
    LimelightHelpers.setLEDMode_ForceBlink("limelight");
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.3);
    timer.start();
   }

   public boolean isFinished() {
       return timer.get() >= 4.0;
   }

   public void end() {
       LimelightHelpers.setLEDMode_ForceOff("limelight");
       driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
   }
}