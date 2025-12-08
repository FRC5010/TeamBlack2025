package frc.robot.blackteam;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.subsystems.LEDStrip;

public class BlackRobot extends GenericRobot {
  private GenericDrivetrain drivetrain;
  private LowerFlyWheel lowerFlyWheel;
  private UpperFlyWheel upperFlyWheel;
  private StateMachine flyWheelStateMachine = new StateMachine(logPrefix);
  private final double SPEED1 = 0.15;
  private final double SPEED2 = 0.16;
  private final double SPEED3 = 0.17;
  private final double SPEED4 = 0.18;
  private final double UPPEROFFSET = 0.01;
  private final AngularVelocity VELOCITY2 = RPM.of(1500);
  private final String STATUS_LED = "status_indicator";
  private final String TEAM_COLORS = "team_colors";

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
    driver.setRightTrigger(driver.createRightTrigger().cubed().deadzone(0.05).scale(0.35));
    driver.setLeftTrigger(driver.createLeftTrigger().cubed().deadzone(0.05).scale(0.35));
    LEDStrip.setSegmentActive(STATUS_LED, false);
    new Trigger(DriverStation::isTeleopEnabled)
        .onTrue(Commands.run(() -> LEDStrip.setSegmentActive(STATUS_LED, true)));
    driver
        .createAButton()
        .whileTrue(
            lowerFlyWheel
                .set(SPEED1)
                .alongWith(upperFlyWheel.set(SPEED1 + UPPEROFFSET))
                .beforeStarting(
                    () ->
                        LEDStrip.changeSegmentPattern(
                            STATUS_LED, LEDStrip.getSolidPattern(Color.kAquamarine)))
                .finallyDo(
                    () ->
                        LEDStrip.changeSegmentPattern(
                            STATUS_LED, LEDStrip.getSolidPattern(Color.kBlack))));
    driver
        .createXButton()
        .whileTrue(
            lowerFlyWheel.setVelocity(VELOCITY2).alongWith(upperFlyWheel.setVelocity(VELOCITY2)));
    driver
        .createYButton()
        .whileTrue(lowerFlyWheel.set(SPEED3).alongWith(upperFlyWheel.set(SPEED3 + UPPEROFFSET)));
            lowerFlyWheel
                .set(SPEED2)
                .alongWith(upperFlyWheel.set(SPEED3 + UPPEROFFSET))
                .beforeStarting(
                    () ->
                        LEDStrip.changeSegmentPattern(
                            STATUS_LED, LEDStrip.getSolidPattern(Color.kAquamarine)))
                .finallyDo(
                    () ->
                        LEDStrip.changeSegmentPattern(
                            STATUS_LED, LEDStrip.getSolidPattern(Color.kRed)));

    driver
        .createYButton()
        .whileTrue(
            lowerFlyWheel
                .set(SPEED3)
                .alongWith(upperFlyWheel.set(SPEED3 + UPPEROFFSET))
                .beforeStarting(
                    () ->
                        LEDStrip.changeSegmentPattern(
                            STATUS_LED, LEDStrip.getSolidPattern(Color.kAquamarine)))
                .finallyDo(
                    () ->
                        LEDStrip.changeSegmentPattern(
                            STATUS_LED, LEDStrip.getSolidPattern(Color.kYellowGreen))));
    driver
        .createBButton()
        .whileTrue(
            lowerFlyWheel
                .set(SPEED4)
                .alongWith(upperFlyWheel.set(SPEED4 + UPPEROFFSET))
                .beforeStarting(
                    () -> {
                      LEDStrip.setSegmentActive(TEAM_COLORS, true);
                      LEDStrip.changeSegmentPattern(TEAM_COLORS, LEDStrip.getRainbowPattern(200));
                    })
                .finallyDo(() -> LEDStrip.setSegmentActive(TEAM_COLORS, false)));

    driver.createBackButton().whileTrue(lowerFlyWheel.systemID());
    driver.createStartButton().whileTrue(upperFlyWheel.systemID());
    driver.createLeftBumper().whileTrue(feeder.setSpeed(-0.5));

    JoystickButton rightBumper = driver.createRightBumper();

    State prep =
        flyWheelStateMachine.addState(
            "prep",
            Commands.print("PREP")
                .andThen(
                    lowerFlyWheel
                        .setVelocity(RPM.of(1000))
                        .alongWith(upperFlyWheel.setVelocity(RPM.of(1000)))));
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
