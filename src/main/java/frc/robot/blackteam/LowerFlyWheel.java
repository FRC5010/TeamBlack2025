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

public class LowerFlyWheel extends GenericSubsystem {
  private AngularVelocity setpoint = RPM.of(0);

  private InterpolatingDoubleTreeMap distanceToVelocityMap =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(0.0, 0.0),
          Map.entry(0.5, 500.0),
          Map.entry(1.0, 1000.0),
          Map.entry(1.5, 1500.0));

  private FlyWheel lowerFlyWheel;

  /** Creates a new Shooter. */
  public LowerFlyWheel() {
    super("lowerflywheel.json");
    lowerFlyWheel = (FlyWheel) devices.get("lowerflywheel");
  }

  public Command setLowerFlyWheelSpeed(double speed) {
    return lowerFlyWheel.set(speed);
  }

  /**
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return lowerFlyWheel.set(dutyCycle);
  }

  // TODO: fix this to use Supplier<AngularVelocity>
  public Command launchToDistance(DoubleSupplier distanceSupplier) {
    return lowerFlyWheel.setSpeed(
        () -> RPM.of(distanceToVelocityMap.get(distanceSupplier.getAsDouble())));
  }

  public Command spinAtSpeed(Supplier<AngularVelocity> speedSupplier) {
    return lowerFlyWheel.setSpeed(speedSupplier);
  }

  public Command joyStickControl(Supplier<Double> speedSupplier) {
    return lowerFlyWheel.set(speedSupplier);
  }

  public Supplier<AngularVelocity> getVelocity() {
    return () -> lowerFlyWheel.getSpeed();
  }

  public BooleanSupplier isNearTarget(AngularVelocity range) {
    return lowerFlyWheel.isNear(setpoint, range);
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
    setpoint = speed;
    return lowerFlyWheel.setSpeed(speed);
  }

  public Command systemID() {
    return lowerFlyWheel.sysId(Volts.of(12), Volts.of(1).per(Second), Seconds.of(3));
  }
}
