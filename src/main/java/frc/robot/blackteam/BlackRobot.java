package frc.robot.blackteam;

import static edu.wpi.first.units.Units.RPM;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class BlackRobot extends GenericRobot {
  private GenericDrivetrain drivetrain;
  private ShooterSubsystem shooterSubsystem;

  public BlackRobot(String directory) {
    super(directory);
    drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    shooterSubsystem = new ShooterSubsystem();
    // NamedCommands.registerCommand("shoot", launchToDistance(20));
  }

  private Command launchToDistance(int i) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'launchToDistance'");
  }
  ShooterSubsystem shootersubsystem = new ShooterSubsystem();
  private final CommandXboxController xboxcontroller = new CommandXboxController(0);
  @Override
    public void configureButtonBindings() {
      xboxcontroller.leftTrigger().whileTrue(shootersubsystem.setVelocity(RPM.of(60)));
      xboxcontroller.rightTrigger().whileTrue(shootersubsystem.setVelocity(RPM.of(300)));
  
      xboxcontroller.x().whileTrue(shootersubsystem.set(0.3));
      xboxcontroller.y().whileTrue(shootersubsystem.set(-0.3));
    }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
    driver.setRightTrigger(driver.createRightTrigger());
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
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'configureButtonBindings'");
  }
}
