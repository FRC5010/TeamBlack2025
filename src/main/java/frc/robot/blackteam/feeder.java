// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.blackteam;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.thethriftybot.ThriftyNova;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
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

/** Add your docs here. */
public class feeder extends GenericSubsystem {
  private final ThriftyNova motor = new ThriftyNova(11);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withControlMode(ControlMode.OPEN_LOOP);

  private final SmartMotorController motorController =
      new NovaWrapper(motor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig feederConfig =
      new FlyWheelConfig(motorController)
          .withDiameter(Inches.of(3))
          .withMass(Pounds.of(1))
          .withTelemetry("FeederMech", TelemetryVerbosity.HIGH)
          .withUpperSoftLimit(RPM.of(5000));

  private final FlyWheel feeder = new FlyWheel(feederConfig);

  public Command setSpeed(double speed) {
    return feeder.set(speed);
  }

  public feeder() {}

  @Override
  public void periodic() {
    feeder.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    feeder.simIterate();
  }
}
