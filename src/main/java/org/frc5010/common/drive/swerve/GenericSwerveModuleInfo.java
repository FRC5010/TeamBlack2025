// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.frc5010.common.drive.swerve.akit.Module;
import swervelib.SwerveModule;
import swervelib.telemetry.SwerveDriveTelemetry;

/** Add your docs here. */
public record GenericSwerveModuleInfo(
    double steerAbsoluteDegrees,
    double steerRelativeDegrees,
    double driveRelativePositionMeters,
    double driveVelocityMetersPerSecond,
    double steerVelocityDegreesPerSecond,
    double expectedSteerDegrees) {

  public GenericSwerveModuleInfo(SwerveModule module) {
    this(
        module.getAbsolutePosition(),
        module.getRelativePosition(),
        module.getDriveMotor().getPosition(),
        module.getDriveMotor().getVelocity(),
        module.getAngleMotor().getVelocity(),
        desiredSteerDegrees(module));
  }

  /**
   * The module's commanded (desired) steer angle in degrees, taken from YAGSL's post-optimization
   * telemetry. Falls back to the measured state angle before any command has been issued.
   *
   * @param module the swerve module
   * @return the desired steer angle in degrees
   */
  private static double desiredSteerDegrees(SwerveModule module) {
    SwerveModuleState[] desired = SwerveDriveTelemetry.desiredStatesObj;
    int index = module.moduleNumber;
    if (desired != null && index >= 0 && index < desired.length && desired[index] != null) {
      return desired[index].angle.getDegrees();
    }
    return module.getState().angle.getDegrees();
  }

  public GenericSwerveModuleInfo(Module module) {
    this(
        module.getAngle().getDegrees(),
        module.getAngle().getDegrees(),
        module.getPosition().distanceMeters,
        module.getVelocityMetersPerSec(),
        0.0,
        module.getAngle().getDegrees());
  }
}
