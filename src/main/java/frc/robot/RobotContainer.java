// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState.Event;
import frc.robot.commands.auto.LaunchNoteCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  RobotState state;

  public static CommandXboxController driverController;
  public static CommandXboxController auxController;

  SendableChooser<Command> autoChooser;

  public RobotContainer() {

    configureNamedCommands();

    state = new RobotState();

    driverController = new CommandXboxController(0);
    auxController = new CommandXboxController(1);

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
  }

  public static CommandXboxController getDriverController() {
    return driverController;
  }

  public static CommandXboxController getAuxController() {
    return auxController;
  }

  public void configureBindings() {

    //set up swerve driving here

    //set up event triggers for states
    driverController.rightTrigger(0.5)
      .onTrue(Commands.runOnce(
        () -> state.setEvent(Event.INTAKE_REQUEST)))
      .onFalse(Commands.runOnce(
        () -> state.setEvent(Event.INTAKE_CANCEL)));

    driverController.leftTrigger(0.5)
      .onTrue(Commands.runOnce(
        () -> state.setEvent(Event.OUTTAKE_REQUEST)))
      .onFalse(Commands.runOnce(
        () -> state.setEvent(Event.INTAKE_CANCEL)));

    Intake.getInstance().getIntakeProxTriggered()
      .onTrue(Commands.runOnce(
        () -> state.setEvent(Event.INTAKE_PROX)));

    Launcher.getInstance().getLauncherProxTriggered()
      .onTrue(Commands.runOnce(
        () -> state.setEvent(Event.LAUNCHER_PROX)));

    auxController.x()
      .onTrue(Commands.runOnce(
        () -> state.setEvent(Event.LAUNCH_SPEAKER_REQUEST)));

    Launcher.getInstance().getAimingCompleteTriggered()
      .onTrue(Commands.runOnce(
        () -> state.setEvent(Event.AIMING_COMPLETE)));

    Launcher.getInstance().getLauncherShotTriggered()
      .onTrue(Commands.runOnce(
        () -> state.setEvent(Event.LAUNCHER_SHOT)));

    auxController.b()
      .onTrue(Commands.runOnce(
        () -> state.setEvent(Event.CLIMB_REQUEST)));

    driverController.a()
      .onTrue(Commands.runOnce(
        () -> state.setEvent(Event.PATHING_REQUEST)));

    Climber.getInstance().getClimbComplete()
      .onTrue(Commands.runOnce(
        () -> state.setEvent(Event.CLIMB_COMPLETE)));

    Swerve.getInstance().getPathCompleteTriggered()
      .onTrue(Commands.runOnce(
        () -> state.setEvent(Event.PATHING_COMPLETE)));

  }

  private void configureNamedCommands() {

    NamedCommands.registerCommand("Launch Note", new LaunchNoteCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
