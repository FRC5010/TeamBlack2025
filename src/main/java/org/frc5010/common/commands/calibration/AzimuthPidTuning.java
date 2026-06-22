// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands.calibration;

import edu.wpi.first.wpilibj.Timer;
import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.drive.swerve.GenericSwerveModuleInfo;
import org.frc5010.common.telemetry.DisplayBoolean;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayString;
import org.frc5010.common.telemetry.DisplayValuesHelper;

/**
 * Live tuning helper for the swerve azimuth (angle) motor PID, focused on calibrating the
 * proportional (P) gain so the modules reach and hold their commanded heading tightly.
 *
 * <p>YAGSL stores the azimuth gains in {@code modules/pidfproperties.json} under {@code "angle"}.
 * This command lets you adjust those gains from the dashboard while it repeatedly steps every
 * module between two azimuth setpoints, so you can watch the response and tighten P until the error
 * settles quickly with minimal overshoot.
 *
 * <p>Dashboard inputs (CONFIG level, editable and persistent):
 *
 * <ul>
 *   <li>{@code Angle P/I/D/F} - the gains applied to every module's azimuth motor. Seeded from the
 *       deployed config when the command starts.
 *   <li>{@code Auto Step} - when true, alternates the setpoint between {@code Step Angle A} and
 *       {@code Step Angle B} every {@code Step Period (s)} seconds; when false, holds {@code Target
 *       Angle}.
 *   <li>{@code Target Angle}, {@code Step Angle A}, {@code Step Angle B}, {@code Step Period (s)}.
 * </ul>
 *
 * <p>Outputs: the worst-case and average absolute azimuth error across modules, plus a per-module
 * error, so you can judge how tightly P is holding. When you are happy with the response, copy the
 * dashboard {@code Angle P} value into {@code pidfproperties.json} and redeploy.
 */
public class AzimuthPidTuning extends GenericCommand {
  private static final String[] MODULE_NAMES = {"frontleft", "frontright", "backleft", "backright"};

  private final GenericSwerveDrivetrain drivetrain;
  private final DisplayValuesHelper display =
      new DisplayValuesHelper(logPrefix, "Azimuth PID Tuning");

  // Tunable gains.
  private final DisplayDouble pGain;
  private final DisplayDouble iGain;
  private final DisplayDouble dGain;
  private final DisplayDouble fGain;

  // Tunable setpoint controls.
  private final DisplayBoolean autoStep;
  private final DisplayDouble targetAngle;
  private final DisplayDouble stepAngleA;
  private final DisplayDouble stepAngleB;
  private final DisplayDouble stepPeriod;

  // Outputs.
  private final DisplayDouble appliedTarget;
  private final DisplayDouble maxError;
  private final DisplayDouble averageError;
  private final DisplayString status;
  private DisplayDouble[] moduleErrors;

  private final Timer stepTimer = new Timer();
  private boolean stepToB = false;

  // Track the last applied gains so the motor controllers are only reconfigured when they change.
  private double lastP = Double.NaN;
  private double lastI = Double.NaN;
  private double lastD = Double.NaN;
  private double lastF = Double.NaN;

  /**
   * Creates a new AzimuthPidTuning command.
   *
   * @param drivetrain the swerve drivetrain whose azimuth PID will be tuned
   */
  public AzimuthPidTuning(GenericSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    pGain = display.makeConfigDouble("Angle P");
    iGain = display.makeConfigDouble("Angle I");
    dGain = display.makeConfigDouble("Angle D");
    fGain = display.makeConfigDouble("Angle F");

    autoStep = display.makeConfigBoolean("Auto Step");
    targetAngle = display.makeConfigDouble("Target Angle");
    stepAngleA = display.makeConfigDouble("Step Angle A");
    stepAngleB = display.makeConfigDouble("Step Angle B");
    stepPeriod = display.makeConfigDouble("Step Period (s)");

    appliedTarget = display.makeDisplayDouble("Applied Target (deg)");
    maxError = display.makeDisplayDouble("Max Abs Error (deg)");
    averageError = display.makeDisplayDouble("Average Abs Error (deg)");
    status = display.makeDisplayString("Status");

    addRequirements(drivetrain);
  }

  private void buildModuleErrorDisplays(int moduleCount) {
    moduleErrors = new DisplayDouble[moduleCount];
    for (int i = 0; i < moduleCount; i++) {
      String name = i < MODULE_NAMES.length ? MODULE_NAMES[i] : "module" + i;
      moduleErrors[i] = display.makeDisplayDouble(name + " Error (deg)");
    }
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
    if (moduleErrors == null) {
      buildModuleErrorDisplays(modules.length);
    }

    // Seed the tunable gains from whatever is currently deployed so the dashboard starts from the
    // real baseline. Defaults for the step controls are only applied when unset.
    double[] pidf = drivetrain.getAzimuthPIDF();
    pGain.setValue(pidf[0]);
    iGain.setValue(pidf[1]);
    dGain.setValue(pidf[2]);
    fGain.setValue(pidf[3]);

    if (stepAngleA.getValue() == 0.0 && stepAngleB.getValue() == 0.0) {
      stepAngleA.setValue(0.0);
      stepAngleB.setValue(90.0);
    }
    if (stepPeriod.getValue() <= 0.0) {
      stepPeriod.setValue(1.0);
    }

    lastP = Double.NaN;
    lastI = Double.NaN;
    lastD = Double.NaN;
    lastF = Double.NaN;
    stepToB = false;
    stepTimer.restart();
    status.setValue("Tuning: adjust Angle P on the dashboard and watch the error.");
  }

  private void applyGainsIfChanged() {
    double p = pGain.getValue();
    double i = iGain.getValue();
    double d = dGain.getValue();
    double f = fGain.getValue();
    if (p != lastP || i != lastI || d != lastD || f != lastF) {
      drivetrain.setAzimuthPIDF(p, i, d, f);
      lastP = p;
      lastI = i;
      lastD = d;
      lastF = f;
    }
  }

  private double resolveTarget() {
    if (!autoStep.getValue()) {
      return targetAngle.getValue();
    }
    double period = Math.max(0.05, stepPeriod.getValue());
    if (stepTimer.hasElapsed(period)) {
      stepToB = !stepToB;
      stepTimer.restart();
    }
    return stepToB ? stepAngleB.getValue() : stepAngleA.getValue();
  }

  @Override
  public void execute() {
    applyGainsIfChanged();

    double target = resolveTarget();
    appliedTarget.setValue(target);
    drivetrain.setAzimuthAngle(target);

    GenericSwerveModuleInfo[] modules = drivetrain.getModulesInfo();
    double worst = 0.0;
    double total = 0.0;
    for (int i = 0; i < modules.length && i < moduleErrors.length; i++) {
      double error = Math.abs(normalize(target - modules[i].steerAbsoluteDegrees()));
      moduleErrors[i].setValue(error);
      worst = Math.max(worst, error);
      total += error;
    }
    maxError.setValue(worst);
    averageError.setValue(modules.length > 0 ? total / modules.length : 0.0);
  }

  @Override
  public void stop(boolean interrupted) {
    stepTimer.stop();
    drivetrain.setAzimuthAngle(0.0);
    status.setValue("Stopped. Copy the tuned Angle P into pidfproperties.json when satisfied.");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
