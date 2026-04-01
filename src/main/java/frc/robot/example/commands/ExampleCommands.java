// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.example.subsystems.ExampleSubsystem;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.telemetry.DisplayString;
import org.frc5010.common.telemetry.DisplayValuesHelper;

/** Add your docs here. */
public class ExampleCommands {
  private StateMachine stateMachine;
  private DisplayString commandState;
  private DisplayValuesHelper DisplayHelper;
  private State intakeState;
  private State lowState;
  private State prepState;
  private State readyState;
  private ExampleSubsystem launcher;
  private GenericDrivetrain drivetrain;
  private Map<String, GenericSubsystem> subsystems;
  private Translation2d target = new Translation2d(Inches.of(182.11), Inches.of(158.84));

  private static enum LauncherState {
    INTAKE,
    LOW_SPEED,
    PREP_SHOOT,
    READY_TO_SHOOT
  }

  private LauncherState requestedState = LauncherState.INTAKE;

  public ExampleCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    DisplayHelper = new DisplayValuesHelper("LauncherCommands", "Values");
    commandState = DisplayHelper.makeDisplayString("Launcher State");

    launcher = (ExampleSubsystem) subsystems.get(ExampleSubsystem.class.getSimpleName());
    drivetrain = (GenericDrivetrain) this.subsystems.get(ConfigConstants.DRIVETRAIN);

    stateMachine = new StateMachine("LauncherStateMachine");
    intakeState = stateMachine.addState("INTAKE", intakeStateCommand());
    lowState = stateMachine.addState("LOW-SPEED", lowStateCommand());
    prepState = stateMachine.addState("PREP-SHOOT", prepStateCommand());
    readyState = stateMachine.addState("READY-TO-SHOOT", readyStateCommand());
    stateMachine.setInitialState(intakeState);
  }

  public void setDefaultCommands(Controller driver, Controller operator) {
    if (launcher != null) {
      stateMachine.addRequirements(launcher);
      launcher.setDefaultCommand(stateMachine);
    }
  }

  public void configureButtonBindings(Controller driver, Controller operator) {
    driver.createRightBumper().onTrue(shouldPrepCommand()).onFalse(shouldIntakeCommand());
    driver.createLeftBumper().onTrue(shouldShootCommand()).onFalse(shouldPrepCommand());
    driver.createAButton().onTrue(shouldIntakeCommand()).onFalse(shouldUseLowSpeed());

    driver.createBButton().whileTrue(launcher.sysIdPivot());

    lowState.switchTo(prepState).when(() -> requestedState == LauncherState.PREP_SHOOT);
    prepState.switchTo(lowState).when(() -> requestedState == LauncherState.LOW_SPEED);

    prepState.switchTo(readyState).when(() -> requestedState == LauncherState.READY_TO_SHOOT);
    readyState.switchTo(prepState).when(() -> requestedState == LauncherState.PREP_SHOOT);

    intakeState.switchTo(lowState).when(() -> requestedState == LauncherState.LOW_SPEED);
    lowState.switchTo(intakeState).when(() -> requestedState == LauncherState.INTAKE);
  }

  public Command shouldPrepCommand() {
    return Commands.runOnce(() -> requestedState = LauncherState.PREP_SHOOT);
  }

  public Command shouldUseLowSpeed() {
    return Commands.runOnce(() -> requestedState = LauncherState.LOW_SPEED);
  }

  public Command shouldShootCommand() {
    return Commands.runOnce(() -> requestedState = LauncherState.READY_TO_SHOOT);
  }

  public Command shouldIntakeCommand() {
    return Commands.runOnce(() -> requestedState = LauncherState.INTAKE);
  }

  private Translation2d getTargetPose() {
    return target.minus(drivetrain.getPoseEstimator().getCurrentPose().getTranslation());
  }

  private Command intakeStateCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> commandState.setValue("Intake")),
        launcher.stopTrackingCommand(),
        launcher.intakeCommand());
  }

  private Command lowStateCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> commandState.setValue("Low Speed")),
        launcher.stopIntakeCommand(),
        launcher.trackTargetCommand(() -> getTargetPose()));
  }

  private Command prepStateCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> commandState.setValue("Prep")),
        launcher.trackTargetCommand(() -> getTargetPose()));
  }

  private Command readyStateCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> commandState.setValue("Ready")),
        launcher.trackTargetCommand(() -> getTargetPose()));
  }
}
