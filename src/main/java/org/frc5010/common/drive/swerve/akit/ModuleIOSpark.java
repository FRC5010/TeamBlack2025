// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.frc5010.common.drive.swerve.akit;

import static org.frc5010.common.drive.swerve.akit.DriveConstants.odometryFrequency;
import static org.frc5010.common.drive.swerve.akit.DriveConstants.turnEncoderPositionFactor;
import static org.frc5010.common.drive.swerve.akit.DriveConstants.turnEncoderVelocityFactor;
import static org.frc5010.common.drive.swerve.akit.DriveConstants.turnMotorCurrentLimit;
import static org.frc5010.common.drive.swerve.akit.DriveConstants.turnPIDMaxInput;
import static org.frc5010.common.drive.swerve.akit.DriveConstants.turnPIDMinInput;
import static org.frc5010.common.drive.swerve.akit.util.SparkUtil.ifOk;
import static org.frc5010.common.drive.swerve.akit.util.SparkUtil.sparkStickyFault;
import static org.frc5010.common.drive.swerve.akit.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.frc5010.common.drive.swerve.AkitSwerveConfig;

/**
 * Module IO implementation for a NEO (Spark Max) drive motor, NEO (Spark Max) turn motor, and an
 * absolute encoder (e.g. Canandmag) read through the turn Spark's absolute-encoder port.
 *
 * <p>Everything per-robot is read from the deploy JSON (via {@link AkitSwerveConfig} / {@link
 * SwerveModuleConstants}): drive/steer CAN IDs, drive gear ratio, drive current limit, the
 * motor/encoder inversions, the absolute encoder zero offset, and the drive/steer closed-loop gains
 * ({@code driveMotorControl}/{@code steerMotorControl} in akit units). Only universal akit
 * constants (encoder conversion factors, the steer wrap range, steer current limit, odometry
 * frequency) come from {@link DriveConstants}.
 */
public class ModuleIOSpark implements ModuleIO {
  /** Absolute encoder zero offset, from the JSON {@code absoluteOffset}. */
  private final Rotation2d zeroRotation;

  // Drive feedforward gains (from the JSON), used in setDriveVelocity.
  private final double driveKs;
  private final double driveKv;

  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIOSpark(
      AkitSwerveConfig config,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    zeroRotation = Rotation2d.fromRotations(constants.EncoderOffset);
    driveKs = constants.DriveMotorGains.kS;
    driveKv = constants.DriveMotorGains.kV;

    // Rotor rotations -> wheel radians, using the JSON drive gear ratio.
    double driveEncoderPositionFactor = 2 * Math.PI / constants.DriveMotorGearRatio;
    double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / constants.DriveMotorGearRatio;

    driveSpark = new SparkMax(constants.DriveMotorId, MotorType.kBrushless);
    turnSpark = new SparkMax(constants.SteerMotorId, MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getAbsoluteEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    var driveConfig = new SparkMaxConfig();
    driveConfig
        .inverted(constants.DriveMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) constants.SlipCurrent)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(constants.DriveMotorGains.kP)
        .d(constants.DriveMotorGains.kD);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(constants.SteerMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(constants.EncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .p(constants.SteerMotorGains.kP)
        .d(constants.SteerMotorGains.kD);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> {
          Rotation2d raw = new Rotation2d(value);
          // Raw reading is logged for offset calibration: with the wheel pointed forward, the
          // reported raw angle (deg) is the value to copy into the module's JSON absoluteOffset.
          inputs.turnRawAbsolutePosition = raw;
          inputs.turnAbsolutePosition = raw.minus(zeroRotation);
          // The steering closed loop runs on the absolute encoder and there is no separate relative
          // turn encoder, so turnPosition mirrors the absolute position. Without this, turnPosition
          // stays at 0, which makes Module.getAngle()/cosineScale think the wheel is always at 0 -
          // scaling drive speed by cos(targetAngle) and producing no drive for 90 deg (strafe).
          inputs.turnPosition = inputs.turnAbsolutePosition;
        });
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }
}
