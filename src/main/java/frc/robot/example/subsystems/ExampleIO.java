// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ExampleIO {
  @AutoLog
  public static class ExampleIOInputs {
    public double shooterSpeedDesired = 0.0;
    public double elevatorHeightDesired = 0.0;
    public Angle hoodAngleDesired = Degrees.of(0.0);
    public Angle turretAngleDesired = Degrees.of(0.0);

    public double shooterSpeedActual = 0.0;
    public double elevatorHeightActual = 0.0;
    public Angle hoodAngleActual = Degrees.of(0.0);
    public Angle turretAngleActual = Degrees.of(0.0);

    public boolean shooterSpeedAtGoal = false;
    public boolean elevatorHeightAtGoal = false;
    public boolean hoodAngleAtGoal = false;
    public boolean turretAngleAtGoal = false;

    public double shooterSpeedError = 0.0;
    public double elevatorHeightError = 0.0;
    public double hoodAngleError = 0.0;
    public double turretAngleError = 0.0;

    public double hoodVelocity = 0.0;
    public double turretVelocity = 0.0;
    public double elevatorVelocity = 0.0;
    public double shooterMotorOutput = 0.0;
    public double elevatorMotorOutput = 0.0;
  }

  public default void updateInputs(ExampleIOInputs inputs) {}

  public default void updateSimulation() {}

  public void setPercentMotor(double output);

  public Command setDutyCycle(double output);

  public void runShooter(double speed);

  public Command setUpperSpeed(AngularVelocity speed);

  public Command setElevatorHeight(double height);

  public void setHoodAngle(Angle angle);

  public void setTurretRotation(Angle angle);

  public AngularVelocity getShooterVelocity();

  public Command sysIdShooter();

  public Command sysIdArm();

  public Command sysIdPivot();

  public Command sysIdTurret();

  public default Command addBallToRobot() {
    return Commands.none();
  }

  public Command launchBall();
}
