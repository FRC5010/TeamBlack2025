package frc.robot.blackteam;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import yams.mechanisms.velocity.FlyWheel;

public class UpperFlyWheel extends GenericSubsystem {
  private AngularVelocity setpoint = RPM.of(0);

  private InterpolatingDoubleTreeMap distanceToVelocityMap =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(0.0, 0.0),
          Map.entry(0.5, 500.0),
          Map.entry(1.0, 1000.0),
          Map.entry(1.5, 1500.0));

  private FlyWheel upperFlyWheel;

  /** Creates a new Shooter. */
  public UpperFlyWheel() {
    super("upperflywheel.json");
    upperFlyWheel = (FlyWheel) devices.get("upperflywheel");
  }

  public Command setUpperFLyWheelSpeed(double speed) {
    return upperFlyWheel.set(speed);
  }

  /**
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return upperFlyWheel.set(dutyCycle);
  }

  // TODO: fix this to use Supplier<AngularVelocity>
  public Command launchToDistance(DoubleSupplier distanceSupplier) {
    return upperFlyWheel.setSpeed(
        () -> RPM.of(distanceToVelocityMap.get(distanceSupplier.getAsDouble())));
  }

  public Command spinAtSpeed(Supplier<AngularVelocity> speedSupplier) {
    return upperFlyWheel.setSpeed(speedSupplier);
  }

  public Command joyStickControl(Supplier<Double> speedSupplier) {
    return upperFlyWheel.set(speedSupplier);
  }

  public Supplier<AngularVelocity> getVelocity() {
    return () -> upperFlyWheel.getSpeed();
  }

  public BooleanSupplier isNearTarget(AngularVelocity range) {
    return upperFlyWheel.isNear(setpoint, range);
  }

  @Override
  public void periodic() {
    upperFlyWheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    upperFlyWheel.simIterate();
  }

  public Command setVelocity(AngularVelocity speed) {
    setpoint = speed;
    return upperFlyWheel.setSpeed(speed);
  }

  public Command systemID() {
    return upperFlyWheel.sysId(Volts.of(12), Volts.of(1).per(Second), Seconds.of(3));
  }
}
