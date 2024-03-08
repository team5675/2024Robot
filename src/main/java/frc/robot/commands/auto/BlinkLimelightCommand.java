package frc.robot.commands.auto;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class BlinkLimelightCommand extends Command {

    Timer time = new Timer();
   public BlinkLimelightCommand() {
       
   }

   @Override
   public void initialize() {
       LimelightHelpers.setLEDMode_ForceBlink("limelight");
       time.reset();
   }

   @Override
   public void execute() {
    time.start();
   }

   @Override
   public boolean isFinished() {
       return time.get() >= 2.0;
   }

   @Override
   public void end(boolean interrupted) {
       LimelightHelpers.setLEDMode_ForceOff("limelight");
   }
}