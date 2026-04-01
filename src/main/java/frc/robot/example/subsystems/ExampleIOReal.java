// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.motors.function.AngularControlMotor;
import org.frc5010.common.motors.function.PercentControlMotor;
import org.frc5010.common.motors.function.VelocityControlMotor;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;

/** Add your docs here. */
public class ExampleIOReal implements ExampleIO {
  protected Map<String, Object> devices;
  protected PercentControlMotor percentMotor;
  protected VelocityControlMotor controlledMotor;
  protected AngularControlMotor angularMotor;
  protected FlyWheel shooter;
  protected Arm arm;
  protected GenericSubsystem parent;
  protected Pivot pivot;

  public ExampleIOReal(Map<String, Object> devices, GenericSubsystem parent) {
    this.devices = devices;
    this.parent = parent;
    this.percentMotor = (PercentControlMotor) devices.get("percent_motor");
    this.controlledMotor = (VelocityControlMotor) devices.get("velocity_motor");
    this.shooter = (FlyWheel) devices.get("Shooter");
    this.arm = (Arm) devices.get("Hood");
    this.pivot = (Pivot) devices.get("Turret");
    this.angularMotor = (AngularControlMotor) devices.get("angular_motor");
  }

  @Override
  public void updateInputs(ExampleIOInputs inputs) {

    angularMotor.periodicUpdate();
  }

  @Override
  public void setPercentMotor(double output) {
    percentMotor.set(output);
  }

  @Override
  public void runShooter(double speed) {
    shooter.getMotor().setDutyCycle(speed);
  }

  @Override
  public Command setUpperSpeed(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  @Override
  public Command setElevatorHeight(double height) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setElevatorHeight'");
  }

  @Override
  public void setHoodAngle(Angle angle) {
    arm.getMotorController().setPosition(angle);
  }

  @Override
  public void setTurretRotation(Angle angle) {}

  public AngularVelocity getShooterVelocity() {
    return shooter.getSpeed();
  }

  public Command sysIdShooter() {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(shooter.getMotor(), parent.getName(), parent),
        5,
        3,
        3);
  }

  public Command sysIdTurret() {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.angleSysIdRoutine(
            pivot.getMotorController(), parent.getName(), parent),
        5,
        3,
        3);
  }

  public Command sysIdArm() {
    return arm.sysId(Volts.of(12), Volts.of(1).per(Seconds), Seconds.of(10));
  }

  public Command sysIdPivot() {
    return pivot.sysId(Volts.of(12), Volts.of(1).per(Seconds), Seconds.of(10));
  }

  public Command launchBall() {
    return shooter.set(0);
  }

  public Command setDutyCycle(double output) {
    return Commands.runOnce(() -> percentMotor.set(output), parent);
  }
}
