package frc.robot.blackteam;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;

public class BlackRobot extends GenericRobot {
  private GenericDrivetrain drivetrain;
  private ShooterSubsystem shooterSubsystem;
  private StateMachine flyWheelStateMachine = new StateMachine(logPrefix);

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

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {

    driver.createRightBumper();
    driver.createLeftBumper();
    driver.LEFT_BUMPER.whileTrue(shooterSubsystem.setVelocity(RPM.of(60)));
    driver.RIGHT_BUMPER.whileTrue(shooterSubsystem.setVelocity(RPM.of(300)));
    driver.createXButton();
    driver.createYButton();
    driver.X_BUTTON.whileTrue(shooterSubsystem.set(0.3));
    driver.Y_BUTTON.whileTrue(shooterSubsystem.set(-0.3));
    driver.createAButton().whileTrue(shooterSubsystem.systemID());
    driver.createAButton().onTrue(shooterSubsystem.setSpeed(0.5));

    JoystickButton rightBumper = driver.createRightBumper();

    State idle =
        flyWheelStateMachine.addState("idle", Commands.print("IDLE").andThen(Commands.idle()));
    State prep =
        flyWheelStateMachine.addState("prep", Commands.print("PREP").andThen(Commands.idle()));
    State fire =
        flyWheelStateMachine.addState("fire", Commands.print("FIRE").andThen(Commands.idle()));

    flyWheelStateMachine.setInitialState(idle);
    idle.switchTo(prep).when(rightBumper);
    prep.switchTo(fire).when(shooterSubsystem.isNearTarget(RPM.of(3000), RPM.of(200)));
    prep.switchTo(idle).when(() -> !rightBumper.getAsBoolean());
    fire.switchTo(idle).when(() -> !rightBumper.getAsBoolean());
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
}
