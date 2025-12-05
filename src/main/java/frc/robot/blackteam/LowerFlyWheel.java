package frc.robot.blackteam;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import yams.exceptions.FlyWheelConfigurationException;
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

  public Command launchToDistance(Supplier<AngularVelocity> distanceSupplier) {
    return lowerFlyWheel.setSpeed(distanceSupplier);
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
    return sysId(Volts.of(12), Volts.of(1).per(Second), Seconds.of(7));
  }

  public Command sysId(Voltage maximumVoltage, Velocity<VoltageUnit> step, Time duration) {
    SysIdRoutine routine = lowerFlyWheel.getMotor().sysId(maximumVoltage, step, duration);
    AngularVelocity max = RPM.of(1000);
    AngularVelocity min = RPM.of(-1000);
    if (lowerFlyWheel.getShooterConfig().getUpperSoftLimit().isPresent()) {
      max = lowerFlyWheel.getShooterConfig().getUpperSoftLimit().get().minus(RPM.of(1));
    } else {
      throw new FlyWheelConfigurationException(
          "FlyWheel upper hard and motor controller soft limit is empty",
          "Cannot create SysIdRoutine.",
          "withSoftLimit(Angle,Angle)");
    }
    if (lowerFlyWheel.getShooterConfig().getLowerSoftLimit().isPresent()) {
      min = lowerFlyWheel.getShooterConfig().getLowerSoftLimit().get().plus(RPM.of(1));
    } else {
      throw new FlyWheelConfigurationException(
          "FlyWheel lower hard and motor controller soft limit is empty",
          "Cannot create SysIdRoutine.",
          "withSoftLimit(Angle,Angle)");
    }
    Trigger maxTrigger = lowerFlyWheel.gte(max);
    Trigger minTrigger = lowerFlyWheel.lte(min);

    Command group =
        Commands.print("Starting SysId")
            .andThen(Commands.runOnce(lowerFlyWheel.getMotor()::stopClosedLoopController))
            .andThen(
                routine
                    .dynamic(Direction.kForward)
                    .until(maxTrigger)
                    .finallyDo(
                        (interrupted) -> {
                          if (maxTrigger.getAsBoolean()) System.err.println("Interrupted");
                          else System.err.println("Forward done");
                        }))
            .andThen(Commands.waitSeconds(3))
            .andThen(
                routine
                    .dynamic(Direction.kReverse)
                    .until(minTrigger)
                    .finallyDo(() -> System.err.println("Reverse done")))
            .andThen(Commands.waitSeconds(3))
            .andThen(
                routine
                    .quasistatic(Direction.kForward)
                    .until(maxTrigger)
                    .finallyDo(() -> System.err.println("Quasistatic forward done")))
            .andThen(Commands.waitSeconds(3))
            .andThen(
                routine
                    .quasistatic(Direction.kReverse)
                    .until(minTrigger)
                    .finallyDo(() -> System.err.println("Quasistatic reverse done")));

    if (lowerFlyWheel.getShooterConfig().getTelemetryName().isPresent()) {
      group = group.andThen(Commands.print(getName() + " SysId test done."));
    }
    return group
        .withName(this.getName() + " SysId")
        .finallyDo(lowerFlyWheel.getMotor()::startClosedLoopController);
  }
}
