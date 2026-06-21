// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands.calibration;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.akit.DriveConstants;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.littletonrobotics.junction.Logger;

/**
 * Interactive, on-robot calibration command for the swerve drive motor velocity PID controllers.
 *
 * <p>The command keeps all modules pointed straight forward and drives them with a closed-loop
 * velocity setpoint that steps between a low and a high value on a fixed period. The drive gains
 * (kP, kI, kD, kS, kV) are exposed as editable, persistent dashboard values, and any change is
 * pushed to every module's drive controller on the fly. The commanded setpoint, the measured
 * average wheel velocity, and the tracking error are published live so the velocity step response
 * can be observed while tuning.
 *
 * <p>Recommended workflow:
 *
 * <ol>
 *   <li>Put the robot on blocks (or in a clear, safe area) and schedule this command in test mode.
 *   <li>Tune the feedforward first (kS, kV) using the steady-state portions of each step so the
 *       error is roughly centered on zero.
 *   <li>Raise kP until the response is crisp without sustained oscillation, then add a little kD to
 *       damp overshoot. Use kI sparingly to remove residual steady-state error.
 *   <li>Copy the values shown on the dashboard back into {@link DriveConstants}.
 * </ol>
 */
public class DriveVelocityPIDCalibration extends GenericCommand {
  private final GenericSwerveDrivetrain drivetrain;
  private final AkitSwerveDrive drive;

  private final DisplayValuesHelper displayValues =
      new DisplayValuesHelper(logPrefix, "Drive PID Calibration");

  // Editable / persistent gains and setpoint shape
  private final DisplayDouble kP;
  private final DisplayDouble kI;
  private final DisplayDouble kD;
  private final DisplayDouble kS;
  private final DisplayDouble kV;
  private final DisplayDouble highSetpoint;
  private final DisplayDouble lowSetpoint;
  private final DisplayDouble stepPeriod;

  // Live readouts
  private final DisplayDouble setpointOut;
  private final DisplayDouble measuredOut;
  private final DisplayDouble errorOut;

  // Snapshot of the gains last pushed to the modules, to detect dashboard edits
  private double appliedKp = Double.NaN;
  private double appliedKi = Double.NaN;
  private double appliedKd = Double.NaN;
  private double appliedKs = Double.NaN;
  private double appliedKv = Double.NaN;

  private final Timer timer = new Timer();

  /**
   * Creates a new DriveVelocityPIDCalibration command.
   *
   * @param swerveDrive the swerve drivetrain subsystem to tune (used for command requirements)
   * @param drive the AdvantageKit swerve drive implementation whose drive gains will be updated
   */
  public DriveVelocityPIDCalibration(GenericSwerveDrivetrain swerveDrive, AkitSwerveDrive drive) {
    this.drivetrain = swerveDrive;
    this.drive = drive;

    kP = makeGain("kP", DriveConstants.driveKp);
    kI = makeGain("kI", 0.0);
    kD = makeGain("kD", DriveConstants.driveKd);
    kS = makeGain("kS", DriveConstants.driveKs);
    kV = makeGain("kV", DriveConstants.driveKv);
    highSetpoint = makeGain("High Setpoint (m/s)", 2.0);
    lowSetpoint = makeGain("Low Setpoint (m/s)", 0.5);
    stepPeriod = makeGain("Step Period (s)", 1.5);

    setpointOut = displayValues.makeDisplayDouble("Setpoint (m/s)");
    measuredOut = displayValues.makeDisplayDouble("Measured (m/s)");
    errorOut = displayValues.makeDisplayDouble("Error (m/s)");

    addRequirements(drivetrain);
  }

  /** Creates a CONFIG-level (editable, persistent) dashboard double seeded with a default. */
  private DisplayDouble makeGain(String name, double defaultValue) {
    DisplayDouble value = displayValues.makeConfigDouble(name);
    // Only seed the default the very first time; afterwards the persisted value wins.
    if (value.getValue() == 0.0) {
      value.setValue(defaultValue);
    }
    return value;
  }

  @Override
  public void init() {
    forceApplyGains();
    timer.restart();
  }

  @Override
  public void execute() {
    applyGainsIfChanged();

    // Step the setpoint between the low and high values so a velocity step response is visible.
    double period = Math.max(0.1, stepPeriod.getValue());
    boolean high = (timer.get() % (2.0 * period)) >= period;
    double setpoint = high ? highSetpoint.getValue() : lowSetpoint.getValue();

    // Keep modules pointed straight forward and command the velocity in closed loop.
    drive.runVelocity(new ChassisSpeeds(setpoint, 0.0, 0.0));

    double measured = averageDriveVelocity();
    double error = setpoint - measured;

    setpointOut.setValue(setpoint);
    measuredOut.setValue(measured);
    errorOut.setValue(error);

    Logger.recordOutput("Calibration/DrivePID/Setpoint", setpoint);
    Logger.recordOutput("Calibration/DrivePID/Measured", measured);
    Logger.recordOutput("Calibration/DrivePID/Error", error);
  }

  @Override
  public void stop(boolean interrupted) {
    drive.stop();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /** Returns the average measured drive velocity across all modules in meters per second. */
  private double averageDriveVelocity() {
    var modules = drive.getModulesInfo();
    double sum = 0.0;
    for (var module : modules) {
      sum += module.driveVelocityMetersPerSecond();
    }
    return modules.length > 0 ? sum / modules.length : 0.0;
  }

  /** Pushes the current dashboard gains to the modules only if they have changed. */
  private void applyGainsIfChanged() {
    if (kP.getValue() != appliedKp
        || kI.getValue() != appliedKi
        || kD.getValue() != appliedKd
        || kS.getValue() != appliedKs
        || kV.getValue() != appliedKv) {
      forceApplyGains();
    }
  }

  /** Unconditionally pushes the current dashboard gains to every module's drive controller. */
  private void forceApplyGains() {
    appliedKp = kP.getValue();
    appliedKi = kI.getValue();
    appliedKd = kD.getValue();
    appliedKs = kS.getValue();
    appliedKv = kV.getValue();
    drive.setDriveGains(appliedKp, appliedKi, appliedKd, appliedKs, appliedKv);
  }
}
