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
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;

public class ShooterSubsystem extends GenericSubsystem {
  private final ThriftyNova motor = new ThriftyNova(10);
  private InterpolatingDoubleTreeMap distanceToVelocityMap =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(0.0, 0.0),
          Map.entry(0.5, 500.0),
          Map.entry(1.0, 1000.0),
          Map.entry(1.5, 1500.0));

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              0.00024509, 0, 0, RPM.of(1000), RotationsPerSecondPerSecond.of(500))
          .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(3, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withSimClosedLoopController(
              0.00024509, 0, 0, RPM.of(1000), RotationsPerSecondPerSecond.of(500))
          .withClosedLoopRampRate(Seconds.of(0.25))
          // .withOpenLoopRampRate(Seconds.of(0.25))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.12521, 0.43498))
          .withFeedforward(new SimpleMotorFeedforward(0, 0.12521, 0.43498))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motorController =
      new NovaWrapper(motor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig shooterConfig =
      new FlyWheelConfig(motorController)
          .withDiameter(Inches.of(4))
          .withMass(Pounds.of(1))
          .withUpperSoftLimit(RPM.of(1000))
          .withLowerSoftLimit(RPM.of(-1000))
          .withSpeedometerSimulation()
          .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  private FlyWheel shooter = new FlyWheel(shooterConfig);
  /**
   * @return Shooter velocity.
   */

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    distanceToVelocityMap.put(0.0, 0.0);
    distanceToVelocityMap.put(0.5, 500.0);
  }

  public Command setSpeed(double speed) {
    return shooter.set(speed);
  }

  /**
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return shooter.set(dutyCycle);
  }

  public Command launchToDistance(DoubleSupplier distanceSupplier) {
    return shooter.setSpeed(
        () -> RPM.of(distanceToVelocityMap.get(distanceSupplier.getAsDouble())));
  }

  public Command spinAtSpeed(DoubleSupplier speedSupplier) {
    return shooter.setSpeed(RPM.of(speedSupplier.getAsDouble()));
  }

  public Supplier<AngularVelocity> getVelocity() {
    return () -> shooter.getSpeed();
  }

  public BooleanSupplier isNearTarget(AngularVelocity expected, AngularVelocity range) {
    return shooter.isNear(expected, range);
  }

  @Override
  public void periodic() {
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  public Command setVelocity(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  public Command systemID() {
    return shooter.sysId(Volts.of(12), Volts.of(1).per(Second), Seconds.of(3));
  }
}
