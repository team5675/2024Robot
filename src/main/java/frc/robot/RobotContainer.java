// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.LaunchNoteCommand;
import frc.robot.commands.auto.NoNoteCommand;
import frc.robot.commands.auto.ShutdownLauncherCommand;
import frc.robot.commands.auto.xLineUp;
import frc.robot.commands.auto.yLineUp;
import frc.robot.commands.auto.HeadingFix;
import frc.robot.commands.auto.ForwardNudge;
import frc.robot.subsystems.Blower;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wristavator;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.auto.BlinkLimelightCommand;
import frc.robot.commands.auto.IntakeCommand;
import frc.robot.commands.auto.LEDCommand;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;



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
    NamedCommands.registerCommand("NO NOTE", new NoNoteCommand());
    
   autoChooser = AutoBuilder.buildAutoChooser("Leave Robot Starting Zone");
    /*AutoSelector.addOption("Leave", new PathPlannerAuto("Leave Robot Starting Zone"));
    AutoSelector.addOption("6 Note", new PathPlannerAuto("I6N Auto"));
    AutoSelector.addOption("2 Note", new PathPlannerAuto("2 Note Auto"));*/
    
    LEDs.getInstance().createBlinkin();
    LEDs.getInstance().setAllianceColor();
    

    driverController = new CommandXboxController(0);
    auxController = new CommandXboxController(1);

    configureBindings();

    //PowerDistribution pdp = new PowerDistribution(0, ModuleType.kRev);
    
    //LaunchNoteCommand = Commands.
    //swerveDrive.setCosineCompensator(false);
   SmartDashboard.putData("Auto Chooser", autoChooser);
   //SmartDashboard.putData(pdp);
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
    driverController.rightTrigger(0.5).and(Launcher.getInstance().getNoteSerialized())
      .onTrue(Commands.run(
        () -> {Intake.getInstance().intakeMotor.set(-0.9);
          Launcher.getInstance().noteHolder.set(-0.8);
        }, Intake.getInstance()))
      .onFalse(Commands.run(
        () -> {Intake.getInstance().intakeMotor.set(0);
        Launcher.getInstance().noteHolder.set(0);}, Intake.getInstance()));

    driverController.leftTrigger(0.5)
      .onTrue(Commands.run(
        () -> {Intake.getInstance().intakeMotor.set(0.9);
        Launcher.getInstance().noteHolder.set(0.8);}, Intake.getInstance()))
      .onFalse(Commands.run(
        () -> {Intake.getInstance().intakeMotor.set(0);
        Launcher.getInstance().noteHolder.set(0);}, Intake.getInstance()));

    auxController.x()
      .onTrue(Commands.run(
        () -> {
          Launcher.getInstance().setRPMSpeaker();

        if(Launcher.getInstance().getLauncherAtRPM().getAsBoolean()) {
          Launcher.getInstance().noteHolder.set(-0.8);
        } else {
          Launcher.getInstance().noteHolder.set(0);
        }
        }, Launcher.getInstance()))
        .onFalse(Commands.run(() -> {Launcher.getInstance().setIdle();
          Launcher.getInstance().noteHolder.set(0);}, Launcher.getInstance()));

    auxController.y()
      .onTrue(Commands.run(
        () -> {
          Blower.getInstance().blowerMotorAmp.set(1);
          Launcher.getInstance().setRPMAmp();

        if(Launcher.getInstance().getLauncherAtRPM().getAsBoolean()) {
          Launcher.getInstance().noteHolder.set(-0.8);
        } else {
          Launcher.getInstance().noteHolder.set(0);
        }
        }, Launcher.getInstance(), Blower.getInstance()))
        .onFalse(Commands.run(() -> {
          Launcher.getInstance().setIdle();
          Launcher.getInstance().noteHolder.set(0);
          Blower.getInstance().blowerMotorAmp.set(0);}, Launcher.getInstance()));

    driverController.a()
          .onTrue(Commands.run(
            () -> {
              Launcher.getInstance().setRPMTrap();
            if(Launcher.getInstance().getLauncherAtRPM().getAsBoolean()) {
              Launcher.getInstance().noteHolder.set(-0.8);
            } else {
              Launcher.getInstance().noteHolder.set(0);
            } 
            }, Launcher.getInstance()))
            .onFalse(Commands.run(() -> {Launcher.getInstance().setIdle();
              Launcher.getInstance().noteHolder.set(0);
            }, Launcher.getInstance()));

    auxController.leftBumper()
      .whileTrue(Commands.run(
        () -> {
        Blower.getInstance().blowerMotorTrapLeft.set(1);
        Blower.getInstance().blowerMotorTrapRight.set(-1);
            }, Blower.getInstance()))
              .whileFalse(Commands.run(() -> {
                Blower.getInstance().blowerMotorTrapLeft.set(0);
              Blower.getInstance().blowerMotorTrapRight.set(0);
            }, Blower.getInstance()));

    driverController.x()
      .onTrue(Commands.run(
        () -> {
          Launcher.getInstance().setRPMSpeaker();

        if(Launcher.getInstance().getLauncherAtRPM().getAsBoolean()) {
          Launcher.getInstance().noteHolder.set(-0.8);
        } else {
          Launcher.getInstance().noteHolder.set(0);
        }
        }, Launcher.getInstance()))
        .onFalse(Commands.run(() -> {Launcher.getInstance().setIdle();
          Launcher.getInstance().noteHolder.set(0);}, Launcher.getInstance()));

    Launcher.getInstance().getNoteSerialized().negate().onTrue(new BlinkLimelightCommand());
    Launcher.getInstance().getNoteSerialized().negate().onTrue(new LEDCommand()).onFalse(Commands.runOnce(() -> LEDs.getInstance().turnOff()));

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

  
    auxController.rightBumper()
        .onTrue(Commands.run(
         () -> {
          Climber.getInstance().raiseClimber();
         }, Climber.getInstance()))
        .onFalse(Commands.run(
          () -> {
            Climber.getInstance().stopClimber();
          }, Climber.getInstance()));

          auxController.a()
        .whileTrue(Commands.run(
         () -> {
          Climber.getInstance().climberRevolutions();
         }, Climber.getInstance()))
        .onFalse(Commands.runOnce(
          () -> {
            Climber.getInstance().stopClimber();
          }, Climber.getInstance()));

          //Elevator
          driverController.rightBumper().onTrue(Commands.run(() -> Wristavator.getInstance().setElevatorHeight(0.2), Wristavator.getInstance())).onFalse(
            Commands.run(() -> Wristavator.getInstance().stopElevator(), Wristavator.getInstance()));
          
          driverController.leftBumper().onTrue(Commands.run(() -> Wristavator.getInstance().setElevatorHeight(0.1), Wristavator.getInstance()))
          .onFalse(Commands.run(() -> Wristavator.getInstance().stopElevator(), Wristavator.getInstance()));

          driverController.povRight().whileTrue(new xLineUp());
          driverController.povUp().whileTrue(new HeadingFix());
          driverController.povDown().whileTrue(new ForwardNudge());
          driverController.povLeft().whileTrue(new yLineUp());

    driverController.b().onTrue(Commands.runOnce(() -> Swerve.getInstance().resetHeading(), Swerve.getInstance()));
          
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
