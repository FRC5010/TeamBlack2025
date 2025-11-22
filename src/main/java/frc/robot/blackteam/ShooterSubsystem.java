package frc.robot.blackteam;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.thethriftybot.ThriftyNova;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;

public class ShooterSubsystem extends GenericSubsystem {
  private final ThriftyNova lowerMotor = new ThriftyNova(10);
  private final ThriftyNova upperMotor = new ThriftyNova(11);
  // private final SparkMax motor = new SparkMax(10, MotorType.kBrushless);
  private InterpolatingDoubleTreeMap distanceToVelocityMap =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(0.0, 0.0),
          Map.entry(0.5, 500.0),
          Map.entry(1.0, 1000.0),
          Map.entry(1.5, 1500.0));

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
          .withSimClosedLoopController(
              0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("LowerShooterMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          // ThriftyNova does not support separate closed loop and open loop ramp rates
          // .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
          .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController lowerMotorController =
      new NovaWrapper(lowerMotor, DCMotor.getNEO(1), motorConfig);
  // new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);
  private final SmartMotorController upperMotorController =
      new NovaWrapper(upperMotor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig lFlyWheelConfig =
      new FlyWheelConfig(lowerMotorController)
          .withDiameter(Inches.of(3))
          .withMass(Pounds.of(2))
          .withSoftLimit(RPM.of(-500), RPM.of(500))
          .withSpeedometerSimulation(RPM.of(750))
          .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  private FlyWheel lowerFlyWheel = new FlyWheel(lFlyWheelConfig);

  private final FlyWheelConfig uFlyWheelConfig =
      new FlyWheelConfig(upperMotorController)
          .withDiameter(Inches.of(3))
          .withMass(Pounds.of(2))
          .withUpperSoftLimit(RPM.of(100000))
          .withLowerSoftLimit(RPM.of(-1000))
          .withSpeedometerSimulation()
          .withTelemetry("UpperShooterMech", TelemetryVerbosity.HIGH);

  private FlyWheel upperFlyWheel = new FlyWheel(uFlyWheelConfig);
  /** Creates a new Shooter. */
  public ShooterSubsystem() {}

  public Command setLowerFLyWheelSpeed(double speed) {
    return lowerFlyWheel.set(speed);
  }

  public Command setUpperFLyWheelSpeed(double speed) {
    return upperFlyWheel.set(speed);
  }

  /**
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return lowerFlyWheel.set(dutyCycle);
  }

  public Command launchToDistance(DoubleSupplier distanceSupplier) {
    return lowerFlyWheel.setSpeed(
        () -> RPM.of(distanceToVelocityMap.get(distanceSupplier.getAsDouble())));
  }

  public Command spinAtSpeed(DoubleSupplier speedSupplier) {
    return lowerFlyWheel.setSpeed(RPM.of(speedSupplier.getAsDouble()));
  }

  public Supplier<AngularVelocity> getVelocity() {
    return () -> lowerFlyWheel.getSpeed();
  }

  public BooleanSupplier isNearTarget(AngularVelocity expected, AngularVelocity range) {
    return lowerFlyWheel.isNear(expected, range);
  }

  @Override
  public void periodic() {
    lowerFlyWheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    lowerFlyWheel.simIterate();
  }

  public Command setVelocity(AngularVelocity speed) {
    return lowerFlyWheel.setSpeed(speed);
  }

  public Command systemID() {
    return lowerFlyWheel.sysId(Volts.of(12), Volts.of(1).per(Second), Seconds.of(3));
  }
}
