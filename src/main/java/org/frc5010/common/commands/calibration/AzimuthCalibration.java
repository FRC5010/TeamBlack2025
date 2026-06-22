// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands.calibration;

import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.drive.swerve.GenericSwerveModuleInfo;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayString;
import org.frc5010.common.telemetry.DisplayValuesHelper;

/**
 * Calibration helper for the swerve azimuth (angle) motors when using the YAGSL implementation.
 *
 * <p>YAGSL stores each module's azimuth zero as an {@code absoluteEncoderOffset} value in the module
 * JSON files (for example {@code src/main/deploy/black_robot/yagsl_swerve/modules/frontleft.json}).
 * Calibrating the azimuth motors means finding those offsets so that the reported azimuth reads 0
 * degrees when the wheel physically points straight forward.
 *
 * <p>Procedure while this command is running:
 *
 * <ol>
 *   <li>The azimuth motors are put into coast mode so the wheels can be turned by hand.
 *   <li>Physically rotate every wheel so it points straight forward (bevel gears all facing the same
 *       direction, per the YAGSL documentation).
 *   <li>Read the "Suggested Offset" value published for each module on the dashboard.
 *   <li>Copy each suggested offset into the {@code absoluteEncoderOffset} field of the matching
 *       module JSON file, then redeploy.
 * </ol>
 *
 * <p>The suggested offset assumes the module JSON currently has {@code absoluteEncoderOffset} set to
 * 0. After updating the JSON and redeploying, run again to verify each module's "Absolute Angle"
 * reads close to 0 when the wheels point forward. A summary is also printed to the console when the
 * command ends.
 */
public class AzimuthCalibration extends GenericCommand {
  /** Module index order matches {@link GenericSwerveDrivetrain} visualization: FL, FR, BL, BR. */
  private static final String[] MODULE_NAMES = {"frontleft", "frontright", "backleft", "backright"};

  private final GenericSwerveDrivetrain drivetrain;
  private final DisplayValuesHelper display =
      new DisplayValuesHelper(logPrefix, "Azimuth Calibration");

  private DisplayDouble[] absoluteAngles;
  private DisplayDouble[] suggestedOffsets;
  private DisplayString instructions;

  private boolean initialized = false;

  /**
   * Creates a new AzimuthCalibration command.
   *
   * @param drivetrain the swerve drivetrain to calibrate
   */
  public AzimuthCalibration(GenericSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    instructions = display.makeDisplayString("Instructions");
    addRequirements(drivetrain);
  }

  private void buildDisplays(int moduleCount) {
    absoluteAngles = new DisplayDouble[moduleCount];
    suggestedOffsets = new DisplayDouble[moduleCount];
    for (int i = 0; i < moduleCount; i++) {
      String name = i < MODULE_NAMES.length ? MODULE_NAMES[i] : "module" + i;
      absoluteAngles[i] = display.makeDisplayDouble(name + " Absolute Angle (deg)");
      suggestedOffsets[i] = display.makeDisplayDouble(name + " Suggested Offset (deg)");
    }
    initialized = true;
  }

  /** Normalizes an angle in degrees to the range (-180, 180]. */
  private static double normalize(double degrees) {
    double angle = degrees % 360.0;
    if (angle > 180.0) {
      angle -= 360.0;
    } else if (angle <= -180.0) {
      angle += 360.0;
    }
    return angle;
  }

  @Override
  public void init() {
    GenericSwerveModuleInfo[] modules = drivetrain.getModulesInfo();
    if (!initialized) {
      buildDisplays(modules.length);
    }
    // Coast the azimuth motors so the wheels can be straightened by hand.
    drivetrain.setMotorBrake(false);
    instructions.setValue("Point all wheels straight forward, then read Suggested Offset.");
  }

  @Override
  public void execute() {
    GenericSwerveModuleInfo[] modules = drivetrain.getModulesInfo();
    for (int i = 0; i < modules.length && i < absoluteAngles.length; i++) {
      double absolute = normalize(modules[i].steerAbsoluteDegrees());
      absoluteAngles[i].setValue(absolute);
      // With the configured offset at 0, the current absolute reading is the offset that makes the
      // wheel's forward position read 0 degrees.
      suggestedOffsets[i].setValue(absolute);
    }
  }

  @Override
  public void stop(boolean interrupted) {
    // Restore brake mode for normal driving.
    drivetrain.setMotorBrake(true);

    GenericSwerveModuleInfo[] modules = drivetrain.getModulesInfo();
    StringBuilder summary =
        new StringBuilder("Azimuth calibration - suggested absoluteEncoderOffset values:\n");
    for (int i = 0; i < modules.length; i++) {
      String name = i < MODULE_NAMES.length ? MODULE_NAMES[i] : "module" + i;
      summary
          .append("  ")
          .append(name)
          .append(".json -> \"absoluteEncoderOffset\": ")
          .append(normalize(modules[i].steerAbsoluteDegrees()))
          .append("\n");
    }
    System.out.println(summary);
    instructions.setValue("Done. See console / dashboard for suggested offsets.");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
