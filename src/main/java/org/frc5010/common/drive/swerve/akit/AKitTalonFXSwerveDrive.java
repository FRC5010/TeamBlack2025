// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.frc5010.common.drive.swerve.akit;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.frc5010.common.drive.pose.DrivePoseEstimator;
import org.frc5010.common.drive.pose.SwerveFunctionsPose;
import org.frc5010.common.drive.swerve.AkitTalonFXSwerveConfig;
import org.frc5010.common.drive.swerve.GenericSwerveModuleInfo;
import org.frc5010.common.drive.swerve.SwerveDriveFunctions;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AKitTalonFXSwerveDrive extends SwerveDriveFunctions {
  // The config doesn't include these constants, so they are declared locally

  final AkitTalonFXSwerveConfig config;
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private SysIdRoutine sysId;
  private Field2d field = new Field2d(); // For visualization in SmartDashboard

  public Supplier<RobotConfig> robotConfigSupplier;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final SwerveDriveKinematics kinematics;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator;

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  public AKitTalonFXSwerveDrive(
      AkitTalonFXSwerveConfig config,
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.config = config;
    kinematics = new SwerveDriveKinematics(getModuleTranslations());
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    this.gyroIO = gyroIO;

    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance(config).start();

    // Configure AutoBuilder for PathPlanner
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.CURRENT_MODE != Mode.SIM);
  }

  /**
   * Returns a SysIdRoutine instance configured for system identification of the drive.
   *
   * <p>If the instance variable sysId is null, a new SysIdRoutine instance is created with the
   * provided SubsystemBase and a default configuration. The default configuration includes a no-op
   * mechanism and a logger which records the state of the SysIdRoutine to the {@link Logger}.
   *
   * @param swerveSubsystem The subsystem to add to the requirements of the SysIdRoutine
   * @return A SysIdRoutine instance configured for system identification of the drive
   */
  protected SysIdRoutine getSysId(SubsystemBase swerveSubsystem) {
    if (null == sysId) {
      sysId =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  null,
                  null,
                  (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
              new SysIdRoutine.Mechanism(
                  (voltage) -> runCharacterization(voltage.in(Volts)), null, swerveSubsystem));
    }
    return sysId;
  }

  @Override
  public Command sysIdDriveMotorCommand(SubsystemBase swerveSubsystem) {
    // Configure SysId
    return sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        .andThen(sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        .andThen(sysIdDynamic(SysIdRoutine.Direction.kForward))
        .andThen(sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  @Override
  public Command sysIdAngleMotorCommand(SubsystemBase swerveSubsystem) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sysIdAngleMotorCommand'");
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, config.getMaxDriveSpeed());

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  @Override
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return config.getMaxDriveSpeed().in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / config.DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(config.FrontLeft.LocationX, config.FrontLeft.LocationY),
      new Translation2d(config.FrontRight.LocationX, config.FrontRight.LocationY),
      new Translation2d(config.BackLeft.LocationX, config.BackLeft.LocationY),
      new Translation2d(config.BackRight.LocationX, config.BackRight.LocationY)
    };
  }

  @Override
  public Field2d getField2d() {
    return field;
  }

  @Override
  public DrivePoseEstimator initializePoseEstimator() {
    return new DrivePoseEstimator(new SwerveFunctionsPose(this));
  }

  @Override
  public ChassisSpeeds getRobotVelocity() {
    return getChassisSpeeds();
  }

  @Override
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeeds(), gyroInputs.yawPosition);
  }

  @Override
  public void drive(ChassisSpeeds velocity, DriveFeedforwards feedforwards) {
    runVelocity(velocity);
  }

  @Override
  public void driveFieldOriented(ChassisSpeeds velocity) {
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(velocity, gyroInputs.yawPosition));
  }

  @Override
  public void driveRobotRelative(ChassisSpeeds velocity) {
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(velocity, gyroInputs.yawPosition));
  }

  @Override
  public void resetEncoders() {}

  @Override
  public double getGyroRate() {
    return Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec);
  }

  @Override
  public void drive(
      ChassisSpeeds robotRelativeVelocity, SwerveModuleState[] states, Force[] feedforwardForces) {
    runVelocity(robotRelativeVelocity);
  }

  @Override
  public SwerveModuleState[] getStates() {
    return getModuleStates();
  }

  @Override
  public GenericSwerveModuleInfo[] getModulesInfo() {
    if (null == moduleInfos) moduleInfos = new GenericSwerveModuleInfo[modules.length];
    for (int i = 0; i < modules.length; i++) {
      moduleInfos[i] = new GenericSwerveModuleInfo(modules[i]);
    }
    return moduleInfos;
  }

  @Override
  public AngularVelocity getMaximumModuleAngleVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getMaximumModuleAngleVelocity'");
  }

  @Override
  public Supplier<RobotConfig> getPPRobotConfigSupplier() {
    return robotConfigSupplier;
  }

  @Override
  public void setPPRobotConfigSupplier(Supplier<RobotConfig> robotConfigSupplier) {
    this.robotConfigSupplier = robotConfigSupplier;
  }

  /**
   * Retrieves the pose of the simulated drivetrain from the MapleSim system.
   *
   * @return The current pose of the simulated drivetrain as a {@link Pose2d}.
   */
  @Override
  public Pose2d getSimPose() {
    return driveSimulation.getSimulatedDriveTrainPose();
  }
}
