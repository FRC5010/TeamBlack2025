// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.blackteam;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import yams.mechanisms.velocity.FlyWheel;

/** Add your docs here. */
public class FeederSubsystem extends GenericSubsystem {
  private FlyWheel feeder;

  /** Creates a new Feeder configured from JSON. */
  public FeederSubsystem() {
    super("feeder.json");
    feeder = (FlyWheel) devices.get("feeder");
  }

  public Command setSpeed(double speed) {
    return feeder.set(speed);
  }

  public Command joyStickControl(Supplier<Double> speedSupplier) {
    return feeder.set(speedSupplier);
  }

  @Override
  public void periodic() {
    feeder.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    feeder.simIterate();
  }
}
