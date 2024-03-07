// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState.Event;
import frc.robot.commands.auto.LaunchNoteCommand;
import frc.robot.commands.auto.ShutdownLauncherCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.auto.IntakeCommand;
import frc.robot.subsystems.Wristavator;

public class RobotContainer {

 
  //SwerveDrive swerveDrive;

  public static CommandXboxController driverController;
  public static CommandXboxController auxController;

   private final SendableChooser<Command> autoChooser;
  //private final SendableChooser<PathPlannerAuto> AutoSelector = new SendableChooser<PathPlannerAuto>();

  public RobotContainer() {
  
    // Calling Swerve.java for the Configuring of the Auto Chooser and Building the Auto Chooser
    // NamedCommands.registerCommand("LaunchNoteCommand", new LaunchNoteCommand());
    // NamedCommands.registerCommand("Intake Command", new IntakeCommand());
    
    Swerve.getInstance();
    
    NamedCommands.registerCommand("LaunchNoteCommand", new LaunchNoteCommand());
    NamedCommands.registerCommand("Intake Command", new IntakeCommand());
    NamedCommands.registerCommand("Shutdown Launcher", new ShutdownLauncherCommand());
    
   autoChooser = AutoBuilder.buildAutoChooser("Leave Robot Starting Zone");
    /*AutoSelector.addOption("Leave", new PathPlannerAuto("Leave Robot Starting Zone"));
    AutoSelector.addOption("6 Note", new PathPlannerAuto("I6N Auto"));
    AutoSelector.addOption("2 Note", new PathPlannerAuto("2 Note Auto"));*/
    

    

    driverController = new CommandXboxController(0);
    auxController = new CommandXboxController(1);

    configureBindings();
    
    //LaunchNoteCommand = Commands.
    //swerveDrive.setCosineCompensator(false);
   SmartDashboard.putData("Auto Chooser", autoChooser);
    //SmartDashboard.putData("Auto Selector", AutoSelector);
  }

  public static CommandXboxController getDriverController() {
    return driverController;
  }

  public static CommandXboxController getAuxController() {
    return auxController;
  }

  public void configureBindings() {
    
    //set up event triggers for states
    driverController.rightTrigger(0.5)
      .onTrue(Commands.runOnce(
        () -> RobotState.getInstance().setEvent(Event.INTAKE_REQUEST)))
      .onFalse(Commands.runOnce(
        () -> RobotState.getInstance().setEvent(Event.INTAKE_CANCEL)));

    driverController.leftTrigger(0.5)
      .onTrue(Commands.runOnce(
        () -> RobotState.getInstance().setEvent(Event.OUTTAKE_REQUEST)))
      .onFalse(Commands.runOnce(
        () -> RobotState.getInstance().setEvent(Event.INTAKE_CANCEL)));

    Launcher.getInstance().getNoteSerialized()
      .onTrue(Commands.runOnce(
        () -> RobotState.getInstance().setEvent(Event.HOLDER_PROX)));

    auxController.x().and(Launcher.getInstance().getProximitySensor())
      .onTrue(Commands.runOnce(
        () -> RobotState.getInstance().setEvent(Event.LAUNCH_SPEAKER_REQUEST)));
    
    auxController.y().and(Launcher.getInstance().getProximitySensor())
      .onTrue(Commands.runOnce(
        () -> RobotState.getInstance().setEvent(Event.LAUNCH_AMP_REQUEST)));

    Launcher.getInstance().getLauncherAtRPM()
      .onTrue(Commands.runOnce(
        () -> RobotState.getInstance().setEvent(Event.AIMING_COMPLETE)));

    Launcher.getInstance().getNoteShot()
      .onTrue(Commands.runOnce(
        () -> RobotState.getInstance().setEvent(Event.LAUNCHER_SHOT)));

   

    if(Launcher.getInstance().getProximitySensor().getAsBoolean()){
      Timer timer = new Timer();
      timer.reset();
      LimelightHelpers.setLEDMode_ForceBlink("limelight");
      driverController.getHID().setRumble(RumbleType.kBothRumble, 0.3);
      timer.start();
      double secondsSinceRun = timer.get();
      
      if(secondsSinceRun > 3){
        LimelightHelpers.setLEDMode_ForceOff("limelight");
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
      }


    //auxController.b()
     // .onTrue(Commands.runOnce(
     //   () -> state.setEvent(Event.CLIMB_REQUEST)));

    // driverController.a()
    //   .onTrue(Commands.runOnce(
    //     () -> state.setEvent(Event.PATHING_REQUEST)));

    // Climber.getInstance().getClimbComplete()
    //   .onTrue(Commands.runOnce(
    //     () -> state.setEvent(Event.CLIMB_COMPLETE)));

    // Swerve.getInstance().getPathCompleteTriggered()
    //   .onTrue(Commands.runOnce(
    //     () -> state.setEvent(Event.PATHING_COMPLETE)));

    // //Zero stuff
    // Wristavator.getInstance().getWristZeroTrigger()
    //   .onTrue(Commands.runOnce(
    //     () -> Wristavator.getInstance()
    //       .setWristZeroAngle(Constants.WristavatorConstants.wristZeroOffset)));

    // Wristavator.getInstance().getElevatorZeroTrigger()
    //   .onTrue(Commands.runOnce(
    //     () -> Wristavator.getInstance()
    //       .setElevatorZeroHeight(Constants.WristavatorConstants.elevatorZeroOffset)));

    auxController.a()
        .onTrue(Commands.runOnce(
         () -> RobotState.getInstance().setEvent(Event.CLIMB_RETRACT_REQUEST)))
        .onFalse(Commands.runOnce(
          () -> RobotState.getInstance().setEvent(Event.CLIMB_CANCEL)));
     auxController.b()
     .onTrue(Commands.runOnce(
         () -> RobotState.getInstance().setEvent(Event.CLIMB_EXTENDED_REQUEST)))
         .onFalse(Commands.runOnce(
          () -> RobotState.getInstance().setEvent(Event.CLIMB_CANCEL)));
      auxController.rightTrigger(0.5)
     .onTrue(Commands.runOnce(
         () -> RobotState.getInstance().setEvent(Event.CLIMB_LOCK_REQUEST)));
          auxController.leftTrigger(0.5)
     .onTrue(Commands.runOnce(
         () -> RobotState.getInstance().setEvent(Event.CLIMB_UNLOCK_REQUEST)));
          
  }
  

 

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  
  public static void rumble() {
   
    driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
    auxController.getHID().setRumble(RumbleType.kBothRumble, 1);
    Timer.delay(1.5);
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
    auxController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }
 
   
    //return Commands.runOnce(swerveDrive.resetRobotPose()).andThen(AutoBuilder.followPath(path));
  
}
