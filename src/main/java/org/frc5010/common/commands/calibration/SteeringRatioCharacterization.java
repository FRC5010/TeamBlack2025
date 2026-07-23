// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands.calibration;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.drive.swerve.GenericSwerveModuleInfo;
import org.frc5010.common.telemetry.DisplayBoolean;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayString;
import org.frc5010.common.telemetry.DisplayValuesHelper;

/**
 * Diagnoses steering (azimuth) modules that do not rotate the commanded amount.
 *
 * <p>The command walks the commanded module azimuth around in small, equal steps (default 45&deg;
 * per step, 32 steps, i.e. four full azimuth revolutions). Because every step is smaller than
 * 90&deg;, {@link edu.wpi.first.math.kinematics.SwerveModuleState#optimize} never flips a module,
 * so every module should physically rotate exactly the commanded amount, continuously, in the same
 * direction.
 *
 * <p>At the end of each step three angles are sampled per module and accumulated as wrapped deltas:
 *
 * <ul>
 *   <li><b>commanded</b> &mdash; the azimuth this command asked for
 *   <li><b>absolute</b> &mdash; {@link GenericSwerveModuleInfo#steerAbsoluteDegrees()}, the
 *       CANcoder / duty-cycle encoder on the azimuth output, i.e. true shaft rotation
 *   <li><b>relative</b> &mdash; {@link GenericSwerveModuleInfo#steerRelativeDegrees()}, the steer
 *       motor's internal encoder scaled by the configured angle conversion factor
 * </ul>
 *
 * Two ratios fall out, both of which are 1.0 on a healthy module:
 *
 * <ul>
 *   <li><b>Rotation Ratio</b> = absolute / commanded. Less than 1.0 means the azimuth physically
 *       moved less than it was told to &mdash; a slipping pulley/belt, a stripped gear, or a loose
 *       encoder magnet. This is the check that catches a module rotating at a different rate than
 *       its siblings.
 *   <li><b>Encoder Ratio</b> = relative / absolute. Not 1.0 means the steer motor turned a
 *       different number of rotations than the azimuth actually moved, i.e. the configured angle
 *       conversion factor (gear ratio) does not match the hardware, or something is slipping
 *       between the motor and the azimuth ring. Multiply the configured angle conversion factor by
 *       the reported <i>Suggested Conv Scale</i> (= absolute / relative) to correct it.
 * </ul>
 *
 * <p>Each module's Rotation Ratio is also compared against the median of all modules, so a single
 * bad module is called out by name even when the absolute numbers look plausible.
 *
 * <p><b>Run this with the robot up on blocks.</b> A small drive velocity is required so the
 * drivetrain actually commands azimuth (YAGSL holds the last angle when the requested speed is near
 * zero), which means the wheels will spin and the robot will drive itself in a spiral if it is on
 * the floor.
 *
 * <p>Note: on the AdvantageKit drivetrain the absolute and relative angles come from the same
 * signal, so Encoder Ratio is always exactly 1.0 there and only Rotation Ratio is meaningful.
 */
public class SteeringRatioCharacterization extends GenericCommand {
  /**
   * Default azimuth change per step, in degrees. Must stay below 90 to avoid state optimization.
   */
  public static final double DEFAULT_STEP_DEGREES = 45.0;

  /** Default number of steps, 32 * 45 degrees = four full azimuth revolutions. */
  public static final int DEFAULT_STEPS = 32;

  /** Default fraction of a full turn a ratio may deviate before being called a fault. */
  public static final double DEFAULT_RATIO_TOLERANCE = 0.05;

  private final DisplayValuesHelper displayValuesHelper =
      new DisplayValuesHelper(logPrefix, "Steering Ratio");

  private final GenericSwerveDrivetrain drivetrain;

  private final double stepDegrees;
  private final int totalSteps;
  private final double ratioTolerance;
  private final double driveSpeedMetersPerSecond;

  private final Time alignTime = Seconds.of(1.5);
  private final Time holdTime = Seconds.of(0.5);
  private final Time finalSettleTime = Seconds.of(1.5);

  // Test sequence, built in init()
  private final List<Double> setpointDegrees = new ArrayList<>();
  private final List<Double> setpointHoldSeconds = new ArrayList<>();
  private int setpointIndex = 0;

  private final Timer holdTimer = new Timer();
  private boolean finished = false;

  // Per-module accumulators
  private double[] lastAbsoluteDegrees;
  private double[] lastRelativeDegrees;
  private double[] cumulativeAbsoluteDegrees;
  private double[] cumulativeRelativeDegrees;
  private double[] worstStepErrorDegrees;
  private double[] worstFollowErrorDegrees;
  private double cumulativeCommandedDegrees = 0.0;

  // Telemetry
  private DisplayDouble[] rotationRatioDisplays;
  private DisplayDouble[] encoderRatioDisplays;
  private DisplayDouble[] conversionScaleDisplays;
  private DisplayDouble[] worstStepErrorDisplays;
  private DisplayDouble[] worstFollowErrorDisplays;
  private DisplayDouble commandedTotalDisplay;
  private DisplayDouble progressDisplay;
  private DisplayBoolean faultDetectedDisplay;
  private DisplayString resultDisplay;

  /**
   * Creates a steering ratio characterization with the default sweep.
   *
   * @param drivetrain the swerve drivetrain to test
   */
  public SteeringRatioCharacterization(GenericSwerveDrivetrain drivetrain) {
    this(drivetrain, DEFAULT_STEP_DEGREES, DEFAULT_STEPS, DEFAULT_RATIO_TOLERANCE, 0.25);
  }

  /**
   * Creates a steering ratio characterization.
   *
   * @param drivetrain the swerve drivetrain to test
   * @param stepDegrees azimuth change per step; must be less than 90 so module state optimization
   *     never flips a module mid-sweep
   * @param totalSteps how many steps to take; more steps means more accumulated rotation and a
   *     tighter ratio estimate
   * @param ratioTolerance how far a ratio may sit from 1.0, or from the median of the modules,
   *     before it is reported as a fault
   * @param driveSpeedMetersPerSecond drive velocity requested while stepping; must be large enough
   *     that the drivetrain actually commands azimuth rather than holding the last angle
   */
  public SteeringRatioCharacterization(
      GenericSwerveDrivetrain drivetrain,
      double stepDegrees,
      int totalSteps,
      double ratioTolerance,
      double driveSpeedMetersPerSecond) {
    this.drivetrain = drivetrain;
    this.stepDegrees = stepDegrees;
    this.totalSteps = totalSteps;
    this.ratioTolerance = ratioTolerance;
    this.driveSpeedMetersPerSecond = driveSpeedMetersPerSecond;

    if (Math.abs(stepDegrees) >= 90.0) {
      throw new IllegalArgumentException(
          "stepDegrees must be less than 90 so module optimization does not flip a module");
    }

    int moduleCount = drivetrain.getModulesInfo().length;
    rotationRatioDisplays = new DisplayDouble[moduleCount];
    encoderRatioDisplays = new DisplayDouble[moduleCount];
    conversionScaleDisplays = new DisplayDouble[moduleCount];
    worstStepErrorDisplays = new DisplayDouble[moduleCount];
    worstFollowErrorDisplays = new DisplayDouble[moduleCount];
    for (int i = 0; i < moduleCount; i++) {
      rotationRatioDisplays[i] = displayValuesHelper.makeDisplayDouble("M" + i + " Rotation Ratio");
      encoderRatioDisplays[i] = displayValuesHelper.makeDisplayDouble("M" + i + " Encoder Ratio");
      conversionScaleDisplays[i] =
          displayValuesHelper.makeDisplayDouble("M" + i + " Suggested Conv Scale");
      worstStepErrorDisplays[i] =
          displayValuesHelper.makeDisplayDouble("M" + i + " Worst Step Error (deg)");
      worstFollowErrorDisplays[i] =
          displayValuesHelper.makeDisplayDouble("M" + i + " Worst Follow Error (deg)");
    }
    commandedTotalDisplay = displayValuesHelper.makeDisplayDouble("Commanded Total (deg)");
    progressDisplay = displayValuesHelper.makeDisplayDouble("Progress (%)");
    faultDetectedDisplay = displayValuesHelper.makeDisplayBoolean("Fault Detected");
    resultDisplay = displayValuesHelper.makeDisplayString("Result");

    addRequirements(drivetrain);
  }

  @Override
  public void init() {
    int moduleCount = drivetrain.getModulesInfo().length;
    lastAbsoluteDegrees = new double[moduleCount];
    lastRelativeDegrees = new double[moduleCount];
    cumulativeAbsoluteDegrees = new double[moduleCount];
    cumulativeRelativeDegrees = new double[moduleCount];
    worstStepErrorDegrees = new double[moduleCount];
    worstFollowErrorDegrees = new double[moduleCount];
    cumulativeCommandedDegrees = 0.0;

    setpointDegrees.clear();
    setpointHoldSeconds.clear();
    // Align to zero first; the sample at the end of this hold is the baseline.
    setpointDegrees.add(0.0);
    setpointHoldSeconds.add(alignTime.in(Seconds));
    for (int step = 1; step <= totalSteps; step++) {
      setpointDegrees.add(step * stepDegrees);
      setpointHoldSeconds.add(holdTime.in(Seconds));
    }
    // Repeat the last setpoint with a long settle so the final sample is not polluted by
    // steering lag; only the final position matters, intermediate lag cancels out.
    setpointDegrees.add(totalSteps * stepDegrees);
    setpointHoldSeconds.add(finalSettleTime.in(Seconds));

    setpointIndex = 0;
    finished = false;

    faultDetectedDisplay.setValue(false);
    resultDisplay.setValue("Running - keep the robot on blocks");
    progressDisplay.setValue(0.0);

    holdTimer.reset();
    holdTimer.start();
  }

  @Override
  public void execute() {
    double commanded = setpointDegrees.get(setpointIndex);
    commandAzimuth(commanded);

    if (!holdTimer.hasElapsed(setpointHoldSeconds.get(setpointIndex))) {
      return;
    }

    sample(commanded, setpointIndex > 0);

    setpointIndex++;
    progressDisplay.setValue(100.0 * setpointIndex / setpointDegrees.size());
    if (setpointIndex >= setpointDegrees.size()) {
      report();
      finished = true;
      return;
    }
    holdTimer.reset();
  }

  /** Points every module at the given azimuth by requesting a robot-relative translation. */
  private void commandAzimuth(double azimuthDegrees) {
    double radians = Math.toRadians(azimuthDegrees);
    drivetrain.drive(
        new ChassisSpeeds(
            driveSpeedMetersPerSecond * Math.cos(radians),
            driveSpeedMetersPerSecond * Math.sin(radians),
            0.0));
  }

  /**
   * Records the current module angles.
   *
   * @param commandedDegrees the azimuth that was being held
   * @param accumulate false for the baseline sample, which only seeds the last-angle arrays
   */
  private void sample(double commandedDegrees, boolean accumulate) {
    GenericSwerveModuleInfo[] modules = drivetrain.getModulesInfo();
    double commandedStep = accumulate ? wrapHalf(commandedDegrees - lastCommandedDegrees()) : 0.0;

    for (int i = 0; i < modules.length; i++) {
      double absolute = modules[i].steerAbsoluteDegrees();
      double relative = modules[i].steerRelativeDegrees();

      if (accumulate) {
        double absoluteStep = wrapHalf(absolute - lastAbsoluteDegrees[i]);
        double relativeStep = wrapHalf(relative - lastRelativeDegrees[i]);
        cumulativeAbsoluteDegrees[i] += absoluteStep;
        cumulativeRelativeDegrees[i] += relativeStep;
        worstStepErrorDegrees[i] =
            Math.max(worstStepErrorDegrees[i], Math.abs(absoluteStep - commandedStep));
        worstFollowErrorDegrees[i] =
            Math.max(worstFollowErrorDegrees[i], Math.abs(wrapHalf(absolute - commandedDegrees)));
      }

      lastAbsoluteDegrees[i] = absolute;
      lastRelativeDegrees[i] = relative;
    }

    if (accumulate) {
      cumulativeCommandedDegrees += commandedStep;
      commandedTotalDisplay.setValue(cumulativeCommandedDegrees);
    }
  }

  /** The setpoint held during the previous sample. */
  private double lastCommandedDegrees() {
    return setpointDegrees.get(Math.max(0, setpointIndex - 1));
  }

  /** Computes the ratios, publishes them, and writes a human readable verdict. */
  private void report() {
    int moduleCount = cumulativeAbsoluteDegrees.length;
    double[] rotationRatios = new double[moduleCount];
    double[] encoderRatios = new double[moduleCount];

    for (int i = 0; i < moduleCount; i++) {
      rotationRatios[i] =
          cumulativeCommandedDegrees == 0.0
              ? Double.NaN
              : cumulativeAbsoluteDegrees[i] / cumulativeCommandedDegrees;
      encoderRatios[i] =
          cumulativeAbsoluteDegrees[i] == 0.0
              ? Double.NaN
              : cumulativeRelativeDegrees[i] / cumulativeAbsoluteDegrees[i];

      rotationRatioDisplays[i].setValue(rotationRatios[i]);
      encoderRatioDisplays[i].setValue(encoderRatios[i]);
      conversionScaleDisplays[i].setValue(1.0 / encoderRatios[i]);
      worstStepErrorDisplays[i].setValue(worstStepErrorDegrees[i]);
      worstFollowErrorDisplays[i].setValue(worstFollowErrorDegrees[i]);
    }

    double medianRotationRatio = median(rotationRatios);
    StringBuilder verdict = new StringBuilder();
    boolean fault = false;

    for (int i = 0; i < moduleCount; i++) {
      List<String> problems = new ArrayList<>();
      if (Double.isNaN(rotationRatios[i]) || Math.abs(rotationRatios[i] - 1.0) > ratioTolerance) {
        problems.add(String.format("rotates %.1f%% of commanded", 100.0 * rotationRatios[i]));
      }
      if (Math.abs(rotationRatios[i] - medianRotationRatio) > ratioTolerance) {
        problems.add(
            String.format(
                "%.1f%% off the other modules", 100.0 * (rotationRatios[i] - medianRotationRatio)));
      }
      if (Double.isNaN(encoderRatios[i]) || Math.abs(encoderRatios[i] - 1.0) > ratioTolerance) {
        problems.add(
            String.format(
                "gearing conversion off by %.1f%% (scale conv factor by %.4f)",
                100.0 * (encoderRatios[i] - 1.0), 1.0 / encoderRatios[i]));
      }
      if (!problems.isEmpty()) {
        fault = true;
        verdict
            .append("M")
            .append(i)
            .append(": ")
            .append(String.join("; ", problems))
            .append(" | ");
      }
    }

    if (!fault) {
      verdict.append(
          String.format(
              "All modules within %.0f%% over %.0f deg commanded",
              100.0 * ratioTolerance, cumulativeCommandedDegrees));
    }

    faultDetectedDisplay.setValue(fault);
    resultDisplay.setValue(verdict.toString());
    log(logPrefix + ": " + verdict);
  }

  /** Wraps a degree delta into (-90, 90], the largest range a single step can legally span. */
  private static double wrapHalf(double degrees) {
    return MathUtil.inputModulus(degrees, -90.0, 90.0);
  }

  private static double median(double[] values) {
    double[] sorted = values.clone();
    Arrays.sort(sorted);
    int middle = sorted.length / 2;
    return sorted.length % 2 == 0 ? (sorted[middle - 1] + sorted[middle]) / 2.0 : sorted[middle];
  }

  @Override
  public void stop(boolean interrupted) {
    drivetrain.stop();
    holdTimer.stop();
    if (interrupted) {
      resultDisplay.setValue("Interrupted before completing the sweep");
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
