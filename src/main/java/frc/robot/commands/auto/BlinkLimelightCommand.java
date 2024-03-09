package frc.robot.commands.auto;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;

public class BlinkLimelightCommand extends Command {

    Timer time = new Timer();
    NetworkTableEntry led = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode");
   public BlinkLimelightCommand() {
       
   }

   @Override
   public void initialize() {
       //LimelightHelpers.setLEDMode_ForceBlink("limelight");
       
       time.reset();
       led.setNumber(2);
    RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
       System.out.println("TUrning LL On");
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
    led.setNumber(0);
    RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
       //LimelightHelpers.setLEDMode_ForceOff("limelight");
       //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setInteger(0);
   }
}