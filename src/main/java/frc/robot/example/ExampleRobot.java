// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.example.commands.ExampleCommands;
import frc.robot.example.subsystems.ExampleSubsystem;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;

/** This is an example robot class. */
public class ExampleRobot extends GenericRobot {
  SwerveConstants swerveConstants;
  GenericDrivetrain drivetrain;
  DisplayValueSubsystem displayValueSubsystem = new DisplayValueSubsystem();
  ExampleSubsystem exampleSubsystem;
  ExampleCommands exampleCommands;

  public ExampleRobot(String directory) {
    super(directory);
    drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    exampleSubsystem = new ExampleSubsystem();
    exampleCommands = new ExampleCommands(subsystems);
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    exampleCommands.configureButtonBindings(driver, operator);
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
    exampleCommands.setDefaultCommands(driver, operator);
    drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
  }

  @Override
  public void initAutoCommands() {
    drivetrain.setAutoBuilder();
  }

  @Override
  public Command generateAutoCommand(Command autoCommand) {
    return drivetrain.generateAutoCommand(autoCommand);
  }

  @Override
  public void buildAutoCommands() {
    super.buildAutoCommands();
    selectableCommand.addOption("Do Nothing", Commands.none());
    drivetrain.addAutoCommands(selectableCommand);
  }
}
