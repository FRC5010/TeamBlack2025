package frc.robot.blackteam;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final double SPEED2 = 0.35;
  private final double SPEED3 = 0.16;
  private final double SPEED4 = 0.17;
  private final double UPPEROFFSET = 0.01;
  private final AngularVelocity VELOCITY2 = RPM.of(1500);
  private final String FEEDER_LED = "left_half";
  private final String SHOOTER_LED = "right_half";

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
    drivetrain.configureButtonBindings(driver, operator);
    driver
        .createRightStickButton()
        .onTrue(Commands.runOnce(() -> drivetrain.toggleFieldOrientedDrive()));
    // driver.setRightTrigger(driver.createRightTrigger().cubed().deadzone(0.05).scale(0.35));
    // driver.setLeftTrigger(driver.createLeftTrigger().cubed().deadzone(0.05).scale(0.35));
    driver
        .createLeftBumper()
        .whileTrue(
            (lowerFlyWheel
                    .set(SPEED1)
                    .alongWith(upperFlyWheel.set(SPEED1 - UPPEROFFSET))
                    .beforeStarting(
                        () -> {
                          LEDStrip.setSegmentActive(SHOOTER_LED, true);
                          LEDStrip.changeSegmentPattern(
                              SHOOTER_LED, LEDStrip.getRainbowPattern(50));
                        }))
                .finallyDo(() -> LEDStrip.setSegmentActive(SHOOTER_LED, false)));
    driver
        .createXButton()
        .whileTrue(
            lowerFlyWheel
                .set(SPEED2)
                .alongWith(upperFlyWheel.set(SPEED2 - UPPEROFFSET))
                .beforeStarting(
                    () -> {
                      LEDStrip.setSegmentActive(SHOOTER_LED, true);
                      LEDStrip.changeSegmentPattern(SHOOTER_LED, LEDStrip.getRainbowPattern(100));
                    })
                .finallyDo(() -> LEDStrip.setSegmentActive(SHOOTER_LED, false)));

    driver
        .createYButton()
        .whileTrue(
            lowerFlyWheel
                .set(SPEED3)
                .alongWith(upperFlyWheel.set(SPEED3 - UPPEROFFSET))
                .beforeStarting(
                    () -> {
                      LEDStrip.setSegmentActive(SHOOTER_LED, true);
                      LEDStrip.changeSegmentPattern(SHOOTER_LED, LEDStrip.getRainbowPattern(100));
                    })
                .finallyDo(() -> LEDStrip.setSegmentActive(SHOOTER_LED, false)));
    driver
        .createRightBumper()
        .whileTrue(
            lowerFlyWheel
                .set(SPEED4)
                .alongWith(upperFlyWheel.set(SPEED4 - UPPEROFFSET))
                .beforeStarting(
                    () -> {
                      LEDStrip.setSegmentActive(SHOOTER_LED, true);
                      LEDStrip.changeSegmentPattern(SHOOTER_LED, LEDStrip.getRainbowPattern(200));
                    })
                .finallyDo(() -> LEDStrip.setSegmentActive(SHOOTER_LED, false)));

    driver
        .createBButton()
        .onFalse(
            lowerFlyWheel
                .set(0)
                .alongWith(upperFlyWheel.set(0))
                .beforeStarting(() -> LEDStrip.setSegmentActive(SHOOTER_LED, false)));

    // driver.createBackButton().whileTrue(lowerFlyWheel.systemID());
    // driver.createStartButton().whileTrue(upperFlyWheel.systemID());
    driver
        .createAButton()
        .whileTrue(
            feeder
                .setSpeed(0.5)
                .beforeStarting(
                    () -> {
                      LEDStrip.setSegmentActive(FEEDER_LED, true);
                      LEDStrip.changeSegmentPattern(
                          FEEDER_LED, LEDStrip.getLaserPattern(Color.kGreen, 0.2, 150));
                    })
                .finallyDo(() -> LEDStrip.setSegmentActive(FEEDER_LED, false)));

    driver.A_BUTTON.onFalse(feeder.setSpeed(0));

    driver
        .createStartButton()
        .whileTrue(
            feeder
                .setSpeed(-0.5)
                .beforeStarting(
                    () -> {
                      LEDStrip.setSegmentActive(FEEDER_LED, true);
                      LEDStrip.changeSegmentPattern(
                          FEEDER_LED, LEDStrip.getLaserPattern(Color.kGreen, 0.2, 150));
                    })
                .finallyDo(() -> LEDStrip.setSegmentActive(FEEDER_LED, false)));

    driver.START_BUTTON.onFalse(feeder.setSpeed(0));

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
    // rightBumper.whileTrue(flyWheelStateMachine);
    // rightBumper.onFalse(
    // lowerFlyWheel
    // .setVelocity(RPM.of(0))
    // .andThen(lowerFlyWheel.set(0))
    // .alongWith(feeder.setSpeed(0)));
  }

  @Override
  public void configureAltButtonBindings(Controller driver, Controller operator) {
    // Test-mode diagnostics: run the deterministic azimuth step test (robot on blocks).
    driver.createAButton().onTrue(drivetrain.azimuthStepTestCommand());
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
    // lowerFlyWheel.setDefaultCommand(lowerFlyWheel.joyStickControl(() ->
    // driver.getRightTrigger()));
    // upperFlyWheel.setDefaultCommand(upperFlyWheel.joyStickControl(() ->
    // driver.getLeftTrigger()));

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
// frontright has CanId 1, 2 and opposite side of RSL and by the battery
// frontleft has CanId 3, 4  and  next to the RS light
// backright has CanId 7, 8 and next to the feeder motor with CanId 12
// backleft has CanId 5, 6  and next to lower flywheel motor with CanId 10
