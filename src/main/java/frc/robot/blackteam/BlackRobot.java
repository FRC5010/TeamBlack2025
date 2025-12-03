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
  private LowerFlyWheel lowerFlyWheel;
  private UpperFlyWheel upperFlyWheel;
  private StateMachine flyWheelStateMachine = new StateMachine(logPrefix);

  public BlackRobot(String directory) {
    super(directory);
    drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    lowerFlyWheel = new LowerFlyWheel();
    upperFlyWheel = new UpperFlyWheel();

    // NamedCommands.registerCommand("shoot", launchToDistance(20));
  }

  private FeederSubsystem feeder = new FeederSubsystem();

  @SuppressWarnings("unused")
  private Command launchToDistance(int i) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'launchToDistance'");
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    driver.setRightTrigger(driver.createRightTrigger().cubed().deadzone(0.05));
    driver.setLeftTrigger(driver.createLeftTrigger().cubed().deadzone(0.05));

    driver.createLeftBumper().whileTrue(lowerFlyWheel.setVelocity(RPM.of(60)));

    driver.createXButton().whileTrue(lowerFlyWheel.set(0.3).alongWith(upperFlyWheel.set(-0.3)));
    driver.createYButton().whileTrue(lowerFlyWheel.set(-0.3).alongWith(upperFlyWheel.set(0.3)));

    driver.createBackButton().whileTrue(lowerFlyWheel.systemID());
    driver.createStartButton().whileTrue(upperFlyWheel.systemID());
    driver.createAButton().whileTrue(feeder.setSpeed(0.5));

    JoystickButton rightBumper = driver.createRightBumper();

    State prep =
        flyWheelStateMachine.addState(
            "prep", Commands.print("PREP").andThen(lowerFlyWheel.setVelocity(RPM.of(1000))));
    State fire =
        flyWheelStateMachine.addState("fire", Commands.print("FIRE").andThen(feeder.setSpeed(0.5)));

    flyWheelStateMachine.setInitialState(prep);
    prep.switchTo(fire).when(lowerFlyWheel.isNearTarget(RPM.of(200)));
    rightBumper.whileTrue(flyWheelStateMachine);
    rightBumper.onFalse(
        lowerFlyWheel
            .setVelocity(RPM.of(0))
            .andThen(lowerFlyWheel.set(0))
            .alongWith(feeder.setSpeed(0)));
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
    lowerFlyWheel.setDefaultCommand(lowerFlyWheel.joyStickControl(() -> driver.getRightTrigger()));
    upperFlyWheel.setDefaultCommand(upperFlyWheel.joyStickControl(() -> driver.getLeftTrigger()));
    drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
    feeder.setDefaultCommand(feeder.joyStickControl(() -> driver.getLeftTrigger()));
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
