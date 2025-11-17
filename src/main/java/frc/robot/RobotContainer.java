// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.blackteam.ShooterSubsystem;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.WpiHelperInterface;
import org.frc5010.common.arch.WpiNetworkTableValuesHelper;
import org.frc5010.common.config.RobotsParser;
import org.frc5010.common.constants.Constants;

public class RobotContainer implements WpiHelperInterface {
  private static final RobotsParser robotsParser = new RobotsParser();
  public static Constants constants;
  private GenericRobot robot;
  private final ShooterSubsystem shootersubsystem = new ShooterSubsystem();

  private final CommandXboxController xboxcontroller = new CommandXboxController(0);

  public RobotContainer() {
    constants = new Constants();

    robot = robotsParser.getRobot();
    shootersubsystem.setDefaultCommand(shootersubsystem.set(0));
    initAutoCommands();
  
    WpiNetworkTableValuesHelper.loadRegisteredToNetworkTables();
  }


  // Just sets up defalt commands (setUpDeftCom)
  public void setupDefaults() {
    robot.setupDefaultCommands();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return robot.getAutonomousCommand();
  }

  public void initAutoCommands() {
    robot.buildAutoCommands();
  }

  public void disabledBehavior() {
    robot.disabledBehavior();
  }
}
