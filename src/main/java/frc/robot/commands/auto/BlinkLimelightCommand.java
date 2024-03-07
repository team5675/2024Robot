package frc.robot.commands.auto;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class BlinkLimelightCommand extends Command {
   public BlinkLimelightCommand() {
       
   }

   @Override
   public void initialize() {
       LimelightHelpers.setLEDMode_ForceBlink("limelight");
   }

   @Override
   public void execute() {
    Timer time = new Timer();
    time.reset();
   }

   @Override
   public boolean isFinished() {
       return this.getTime() >= 2.0;
   }

   @Override
   public void end(boolean interrupted) {
       LimelightHelpers.setLEDMode_ForceOff("limelight");
   }
}